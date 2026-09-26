import cv2
import sys
import time
import logging
import visionn_model


logger = None
#logging._warn_preinit_stderr = 0

### logger ###

logger_stdout = None
logger_stderr = None

# https://stackoverflow.com/questions/19425736/how-to-redirect-stdout-and-stderr-to-logger-in-python
#class LoggerWriter:
#    def __init__(self, level):
#        # self.level is really like using log.debug(message)
#        # at least in my case
#        self.level = level
#
#    def write(self, message):
#        # if statement reduces the amount of newlines that are
#        # printed to the logger
#        if message != '\n':
#            self.level(message)
#
#    def flush(self):
#        # create a flush method so things can be flushed when
#        # the system wants to. Not sure if simply 'printing'
#        # sys.stderr is the correct way to do it, but it seemed
#        # to work properly for me.
#        self.level(sys.stderr)

def logger_init(name, filename):
    log = logging.getLogger(name)
    log.setLevel(logging.DEBUG)

    handler = logging.FileHandler(filename, mode='a', encoding=None, delay=False)
    handler.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s %(levelname)s [%(name)s] %(message)s')
    handler.setFormatter(formatter)

    log.addHandler(handler)
    log.info('========================================================');

    # redirect stdout/stderr to log file - not working with tensorflow
    logger_stdout = sys.stdout
    logger_stderr = sys.stderr
    #sys.stdout = LoggerWriter(log.debug)
    #sys.stderr = LoggerWriter(log.warning)
    return log

def logger_close():
    sys.stdout = logger_stdout
    sys.stderr = logger_stderr

### mtime ###

def mtime_begin():
    return int(time.time() * 1000)

def mtime_delta(t):
    # return time difference in milliseconds
    return int(mtime_begin() - t)

def mtime_delta2(t1, t2):
    return int(t2 - t1)

def mtime_end(ss, t):
    logger.debug('visionn_server::mtime(): m="' + ss + '", dt=' + str(mtime_delta(t)))


### main ###

logger = logger_init('main', 'visionn_test2.log')

visionn_model.model_init(logger)

img = cv2.imread('img_input/testing_image.png')

pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)

pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)

pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)

cv2.imwrite('img_output/testing_mask.png', pred)
