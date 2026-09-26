import logging
import cv2
import visionn_model

# NOTE: the legacy visionn_test.py called model_init() with no argument at
# all -- doesn't work here, model_init(log) always needs a logger (matches
# visionn_model.py's actual signature, same as the legacy TF version's).
logger = logging.getLogger('visionn_test')
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)

visionn_model.model_init(logger)

img = cv2.imread('img_input/testing_image.png')

pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)

# cv2.imwrite('img_output/testing_mask.png', pred)
