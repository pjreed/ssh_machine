# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import asyncio
from asyncio import Event
from logging import Logger

import asyncssh
import traceback
from typing import Optional, cast

from launch import SomeActionsType, EventHandler
from launch.actions import OpaqueFunction
from launch.event_handlers import OnShutdown
from launch.events.process import ProcessStarted, ProcessExited, ShutdownProcess, \
    SignalProcess
from launch.launch_context import LaunchContext
from launch.machine import Machine
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import is_a_subclass

import launch


class SshClientSession(asyncssh.SSHClientSession):
    def __init__(self, logger: Logger, context: LaunchContext, process_event_args=None):
        self.__logger = logger
        self.__context = context
        self.__process_event_args = process_event_args

    def connection_made(self, chan):
        self.__logger.info("connection_made")

    def data_received(self, data, datatype):
        self.__logger.info("data_received: %s" % str(data))

    def connection_lost(self, exc):
        self.__logger.info("connection_lost: %s" % exc)


class SshMachine(Machine):
    """Describes a machine for remotely launching ROS nodes."""

    def __init__(self, *,
                 hostname: SomeSubstitutionsType,
                 env: Optional[SomeSubstitutionsType] = None,
                 **kwargs) -> None:
        """Initialize a machine description."""
        super().__init__(**kwargs)
        self.__hostname = hostname
        self.__env = env
        self.__conn = None
        self.__chan = None
        self.__logger = None
        self.__first_run = True
        self.__connection_ready = asyncio.Event()

    @property
    def hostname(self) -> Substitution:
        return self.__hostname

    @property
    def env(self):
        return self.__env

    def __on_signal_process_event(self, event: Event, context: LaunchContext):
        if self.__chan:
            typed_event = cast(SignalProcess, context.locals.event)
            self.__logger.info("signals don't work on OpenSSH < 7.9")
            self.__chan.signal(typed_event.signal_name)

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        try:
            if self.__chan:
                self.__logger.debug("Killing all jobs")
                self.__chan.write('kill $(jobs -p)')
                self.__chan.write_eof()
                self.__chan.close()
            self.__logger.debug("Closing SSH connection")
            self.__conn.close()
        except Exception:
            self.__logger.error("Exception when shutting down channel: %s" % traceback.format_exc())

    async def execute_process(self,
                              process_event_args: None,
                              log_cmd: False,
                              emulate_tty: False,
                              shell: False,
                              cleanup_fn: lambda: False,
                              context: LaunchContext) -> None:
        if process_event_args is None:
            raise RuntimeError('process_event_args unexpectedly None')
        cmd = process_event_args['cmd']
        cwd = process_event_args['cwd']
        env = process_event_args['env']

        if not self.__logger:
            self.__logger = launch.logging.get_logger(process_event_args['name'])

            event_handlers = [
                EventHandler(
                    matcher=lambda event: is_a_subclass(event, ShutdownProcess),
                    entities=OpaqueFunction(function=self.__on_shutdown),
                ),
                EventHandler(
                    matcher=lambda event: is_a_subclass(event, SignalProcess),
                    entities=OpaqueFunction(function=self.__on_signal_process_event),
                ),
                OnShutdown(on_shutdown=self.__on_shutdown)
            ]

            self.__logger.info("Registering event handlers")
            for handler in event_handlers:
                context.register_event_handler(handler)

        if log_cmd:
            self.__logger.info("process details: cmd=[{}], cwd='{}', custom_env?={}".format(
                ', '.join(cmd), cwd, 'True' if env is not None else 'False'
            ))

        self.__logger.info("Executing process")

        process_event_args['pid'] = 0
        await context.emit_event(ProcessStarted(**process_event_args))

        try:
            remote_cmd = ""
            if self.__env:
                remote_cmd = "%s;" % self.__env
            remote_cmd = "%s%s" % (remote_cmd, ' '.join(cmd))
            self.__logger.info("Remote cmd: %s" % remote_cmd)

            def create_session():
                return SshClientSession(self.__logger, context, process_event_args)

            if self.__first_run:
                self.__first_run = False
                self.__conn = await asyncssh.connect(self.__hostname)
                self.__chan, session = await self.__conn.create_session(
                    create_session,
                    encoding='utf8')
                if self.__env:
                    self.__chan.write(self.__env)
                    self.__chan.write('\n')
                self.__connection_ready.set()

            await self.__connection_ready.wait()
            if self.__chan:
                self.__chan.write(' '.join(cmd) + ' &\n')

                self.__logger.debug("Waiting for SSH channel to close")
                await self.__chan.wait_closed()

                await context.emit_event(ProcessExited(
                    returncode=self.__chan.get_exit_status(),
                    **process_event_args))

                self.__logger.info("SSH connection exiting")
            else:
                self.__logger.error("SSH channel wasn't ready")
        except Exception:
            self.__logger.error('exception occurred while executing process:\n{}'.format(
                traceback.format_exc()
            ))
        finally:
            cleanup_fn()
