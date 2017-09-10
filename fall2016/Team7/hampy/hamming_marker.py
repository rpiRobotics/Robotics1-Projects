import cv2

from numpy import array, mean, binary_repr, zeros
from numpy.random import randint
from scipy.ndimage import zoom
from PIL import Image

from hampy.hamming import encode, msg_size, data_size

marker_size = msg_size + 2
max_id = 2 ** (msg_size * data_size)


class HammingMarker(object):
    def __init__(self, id, contours=None, img_size=None):
        if not 0 <= id < max_id:
            msg = 'Id must be {} <= id < {} (id={})'.format(0, max_id, id)
            raise ValueError(msg)

        self.id = id
        self.hamming_code = self._encode_id(self.id)
        self.contours = contours
        self.size = list(reversed(img_size)) if img_size is not None else None

    def __repr__(self):
        return '<Marker id={} center={}>'.format(self.id, self.center)

    @property
    def center(self):
        if self.contours is None:
            return None

        return mean(self.contours, axis=0).flatten()

    @property
    def normalized_center(self):
        return ((array(self.center, dtype=float) / self.size) - [0.5, 0.5]) * 2

    def toimage(self, size=marker_size, output=None):
        img = zeros((marker_size, marker_size))
        img[1:-1, 1:-1] = self.hamming_code
        img = 1 - img

        # scale = size / float(marker_size)
        # return zoom(img, zoom=scale, output=output, order=0)

        cv2.imwrite('/tmp/bob.png', img)
        im = Image.open('/tmp/bob.png')
        im2 = im.resize((size, size))
        im2.save('/tmp/bob.png')
        return cv2.imread('/tmp/bob.png')

    def draw_contour(self, img, color=(0, 255, 0), linewidth=5):
        cv2.drawContours(img, [self.contours], -1, color, linewidth)

    @classmethod
    def generate(cls):
        return HammingMarker(id=randint(max_id))

    def _encode_id(self, id):
        s = binary_repr(id, width=msg_size * data_size)
        B = array(list(s), dtype=int).reshape(msg_size, data_size)
        return encode(B)
