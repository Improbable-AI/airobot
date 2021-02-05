import logging

import colorlog


class Logger:
    """
    A logger class.

    Args:
        log_level (str): the following modes are supported:
            `debug`, `info`, `warn`, `error`, `critical`.
    """

    def __init__(self, log_level):
        formatter = colorlog.ColoredFormatter(
            "%(log_color)s[%(levelname)s]%(reset)s[%(asctime)s]: "
            "%(message_log_color)s%(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            reset=True,
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'red,bg_white',
            },
            secondary_log_colors={
                'message': {
                    'DEBUG': 'cyan',
                    'INFO': 'green',
                    'WARNING': 'yellow',
                    'ERROR': 'red',
                    'CRITICAL': 'red'
                }
            },
            style='%'
        )

        handler = colorlog.StreamHandler()
        handler.setFormatter(formatter)

        self.logger = colorlog.getLogger('AIRobot')
        self.logger.addHandler(handler)
        self.set_level(log_level)

    def debug(self, msg):
        """
        Logging debug information

        Args:
            msg (str): message to log
        """
        self.logger.debug(msg)

    def info(self, msg):
        """
        Logging info information

        Args:
            msg (str): message to log
        """
        self.logger.info(msg)

    def warning(self, msg):
        """
        Logging warning information

        Args:
            msg (str): message to log
        """
        self.logger.warning(msg)

    def error(self, msg):
        """
        Logging error information

        Args:
            msg (str): message to log
        """
        self.logger.error(msg)

    def critical(self, msg):
        """
        Logging critical information

        Args:
            msg (str): message to log
        """
        self.logger.critical(msg)

    def set_level(self, log_level):
        """
        Set logging level

        Args:
            log_level (str): the following modes are supported:
                `debug`, `info`, `warn`, `error`, `critical`

        """
        if 'debug' in log_level:
            self.log_level = logging.DEBUG
        elif 'info' in log_level:
            self.log_level = logging.INFO
        elif 'warn' in log_level:
            self.log_level = logging.WARNING
        elif 'error' in log_level:
            self.log_level = logging.ERROR
        elif 'critical' in log_level:
            self.log_level = logging.CRITICAL
        else:
            raise ValueError('Unknown logging '
                             'level: %s' % log_level)
        self.logger.setLevel(self.log_level)


if __name__ == '__main__':
    ai_logger = Logger('debug')
    ai_logger.debug("A quirky message only developers care about")
    ai_logger.info("Curious users might want to know this")
    ai_logger.warning("Something is wrong and any user should be informed")
    ai_logger.error("Serious stuff, this is red for a reason")
    ai_logger.critical("OH NO everything is on fire")
