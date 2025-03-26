import logging

logging.getLogger(__name__).addHandler(logging.NullHandler())


def add_stderr_logger(level: int = logging.DEBUG) -> logging.StreamHandler:
    """
    Helper for quickly adding a StreamHandler to the logger.

    :return: Returns the handler after adding it.
    :rtype: logging.StreamHandler
    """
    # This method needs to be in this __init__.py to get the __name__ correct
    # even if this library is vendored within another package.
    logger = logging.getLogger(__name__)
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(handler)
    logger.setLevel(level)
    logger.debug(f"Added a stderr logging handler to logger: {__name__}")
    return handler
