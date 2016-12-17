from numpy import array, dot

hamming_code = (7, 4)
msg_size, data_size = hamming_code

parity_bits = [0, 1, 3]
data_bits = [2, 4, 5, 6]

G = array(((1, 1, 0, 1),
           (1, 0, 1, 1),
           (1, 0, 0, 0),
           (0, 1, 1, 1),
           (0, 1, 0, 0),
           (0, 0, 1, 0),
           (0, 0, 0, 1)))

H = array(((0, 0, 0, 1, 1, 1, 1),
           (0, 1, 1, 0, 0, 1, 1),
           (1, 0, 1, 0, 1, 0, 1)))


def encode(B):
    """ Encode data using Hamming(7, 4) code.

    E.g.:
        encode([0, 0, 1, 1])

        encode([[0, 0, 0, 1],
                [0, 1, 0, 1]])

    :param array B: binary data to encode (must be shaped as (4, ) or (-1, 4)).

    """
    B = array(B)

    flatten = False
    if len(B.shape) == 1:
        flatten = True
        B = B.reshape(1, -1)

    if B.shape[1] != data_size:
        raise ValueError('Data must be shaped as (4, ) or (-1, 4)')

    C = dot(G, B.T).T % 2

    if flatten:
        C = C.flatten()

    return C


def decode(C):
    """ Decode data using Hamming(7, 4) code.

    E.g.:
        decode([1, 0, 0, 0, 0, 1, 1])

        encode([[1, 1, 0, 1, 0, 0, 1],
                [0, 1, 0, 0, 1, 0, 1]])

    :param array C: binary data to code (must be shaped as (7, ) or (-1, 7)).

    """
    C = array(C)

    flatten = False
    if len(C.shape) == 1:
        flatten = True
        C = C.reshape(1, -1)

    if C.shape[1] != msg_size:
        raise ValueError('Data must be shaped as (7, ) or (-1, 7)')

    if 1 in dot(H, C.T).T % 2:
        raise ValueError('Incorrect code given as input.')

    B = C[:, data_bits]

    if flatten:
        B = B.flatten()

    return B


if __name__ == '__main__':
    import unittest

    from numpy import all
    from numpy.random import randint

    class TestHammingCodeDecode(unittest.TestCase):
        def setUp(self):
            self.b1 = array([0, 0, 1, 1])
            self.b2 = [[0, 0, 0, 1],
                       [0, 1, 0, 1]]

            self.c1 = [1, 0, 0, 0, 0, 1, 1]
            self.c2 = array([[1, 1, 0, 1, 0, 0, 1],
                             [0, 1, 0, 0, 1, 0, 1]])

        def test_code(self):
            self.assertTrue(all(encode(self.b1) == self.c1))
            self.assertTrue(all(encode(self.b2) == self.c2))

        def test_decode(self):
            decode(self.c2)

        def test_code_decode(self):
            B = randint(2, size=(1000, data_size))
            self.assertTrue(all(decode(encode(B)) == B))

    unittest.main()
