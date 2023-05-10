from akro import Box as AkroBox
import tensorflow as tf


class Box(AkroBox):
    def new_tensor_variable(self, name, extra_dims, flatten=True):
        if flatten:
            return tf.compat.v1.placeholder(tf.float32, shape=[None] * extra_dims + [self.flat_dim], name=name)
        return tf.compat.v1.placeholder(tf.float32, shape=[None] * extra_dims + list(self.shape), name=name)

    @property
    def dtype(self):
        return tf.float32
