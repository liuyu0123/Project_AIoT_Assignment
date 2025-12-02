import sys
import logging


def get_logger(name, log_level=logging.INFO, log_format=None, log_file=None):
    """
    获取一个配置好的日志记录器。

    :param name: 日志记录器的名称
    :param log_level: 日志级别，默认为 logging.INFO
    :param log_format: 日志格式，默认为 '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    :param log_file: 日志文件路径，如果不提供则输出到控制台
    :return: 配置好的日志记录器
    """
    # 创建日志记录器
    logger = logging.getLogger(name)
    logger.setLevel(log_level)

    if logger.handlers:
        return logger

    # 设置日志格式
    if log_format is None:
        log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    formatter = logging.Formatter(log_format)

    # 如果提供了日志文件路径，则将日志输出到文件
    if log_file:
        handler = logging.FileHandler(log_file)
    else:
        handler = logging.StreamHandler(sys.stdout)

    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger