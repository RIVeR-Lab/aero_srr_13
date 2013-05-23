"""autogenerated by genpy from aero_srr_snowfence_detector/Snowfence.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Snowfence(genpy.Message):
  _md5sum = "792f7eb8ac07e9462b5985de192cc93e"
  _type = "aero_srr_snowfence_detector/Snowfence"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Snowfence locations
float32[40] X
float32[40] Y
float32[40] width
float32[40] height

"""
  __slots__ = ['X','Y','width','height']
  _slot_types = ['float32[40]','float32[40]','float32[40]','float32[40]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       X,Y,width,height

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Snowfence, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.X is None:
        self.X = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.Y is None:
        self.Y = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.width is None:
        self.width = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.height is None:
        self.height = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    else:
      self.X = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.Y = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.width = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.height = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_40f.pack(*self.X))
      buff.write(_struct_40f.pack(*self.Y))
      buff.write(_struct_40f.pack(*self.width))
      buff.write(_struct_40f.pack(*self.height))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 160
      self.X = _struct_40f.unpack(str[start:end])
      start = end
      end += 160
      self.Y = _struct_40f.unpack(str[start:end])
      start = end
      end += 160
      self.width = _struct_40f.unpack(str[start:end])
      start = end
      end += 160
      self.height = _struct_40f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.X.tostring())
      buff.write(self.Y.tostring())
      buff.write(self.width.tostring())
      buff.write(self.height.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 160
      self.X = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=40)
      start = end
      end += 160
      self.Y = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=40)
      start = end
      end += 160
      self.width = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=40)
      start = end
      end += 160
      self.height = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=40)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_40f = struct.Struct("<40f")
