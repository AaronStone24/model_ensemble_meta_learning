


# from rllab_maml.spaces.box import Box as TheanoBox
from akro import Box as AkroBox
import tensorflow as tf


class Box(AkroBox):
    def new_tensor_variable(self, name, extra_dims):
        return tf.placeholder(tf.float32, shape=[None] * extra_dims + [self.flat_dim], name=name)
