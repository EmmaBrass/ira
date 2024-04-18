
import datetime
import logging
import os
import sys

def setup_logger(name):
    # Set up logging.
    logger = logging.getLogger(name)
    # Set log level.
    logger.setLevel(logging.INFO)
    # Define file handler and set formatter.
    log_dir = 'logs'  # Change this to the desired directory name
    os.makedirs(log_dir, exist_ok=True)  # Ensure the directory exists
    log_file_path = os.path.join(
        log_dir, 
        f'logfile_{datetime.datetime.now().strftime("%I-%M%p_%B-%d-%Y")}.log'
    )
    file_handler = logging.FileHandler(
        log_file_path
    )
    stream_handler = logging.StreamHandler(sys.stdout)
    formatter    = logging.Formatter(
        '%(asctime)s : %(levelname)s : %(name)s : %(module)s : %(message)s'
    )
    file_handler.setFormatter(formatter)
    stream_handler.setFormatter(formatter)
    # Add handlers to logger.
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    return logger

