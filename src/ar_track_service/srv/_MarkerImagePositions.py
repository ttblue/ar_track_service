"""autogenerated by genpy from ar_track_service/MarkerImagePositionsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class MarkerImagePositionsRequest(genpy.Message):
  _md5sum = "81f00002e2dfcb40c021e4ab5c5722a4"
  _type = "ar_track_service/MarkerImagePositionsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/Image img
bool track

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in src/image_encodings.cpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['img','track']
  _slot_types = ['sensor_msgs/Image','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       img,track

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MarkerImagePositionsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.img is None:
        self.img = sensor_msgs.msg.Image()
      if self.track is None:
        self.track = False
    else:
      self.img = sensor_msgs.msg.Image()
      self.track = False

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
      _x = self
      buff.write(_struct_3I.pack(_x.img.header.seq, _x.img.header.stamp.secs, _x.img.header.stamp.nsecs))
      _x = self.img.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.img.height, _x.img.width))
      _x = self.img.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.img.is_bigendian, _x.img.step))
      _x = self.img.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.track))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.img is None:
        self.img = sensor_msgs.msg.Image()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.img.header.seq, _x.img.header.stamp.secs, _x.img.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.img.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.img.height, _x.img.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.encoding = str[start:end].decode('utf-8')
      else:
        self.img.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.img.is_bigendian, _x.img.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.data = str[start:end].decode('utf-8')
      else:
        self.img.data = str[start:end]
      start = end
      end += 1
      (self.track,) = _struct_B.unpack(str[start:end])
      self.track = bool(self.track)
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
      _x = self
      buff.write(_struct_3I.pack(_x.img.header.seq, _x.img.header.stamp.secs, _x.img.header.stamp.nsecs))
      _x = self.img.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.img.height, _x.img.width))
      _x = self.img.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.img.is_bigendian, _x.img.step))
      _x = self.img.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.track))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.img is None:
        self.img = sensor_msgs.msg.Image()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.img.header.seq, _x.img.header.stamp.secs, _x.img.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.img.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.img.height, _x.img.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.encoding = str[start:end].decode('utf-8')
      else:
        self.img.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.img.is_bigendian, _x.img.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.img.data = str[start:end].decode('utf-8')
      else:
        self.img.data = str[start:end]
      start = end
      end += 1
      (self.track,) = _struct_B.unpack(str[start:end])
      self.track = bool(self.track)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_2I = struct.Struct("<2I")
_struct_BI = struct.Struct("<BI")
"""autogenerated by genpy from ar_track_service/MarkerImagePositionsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import ar_track_alvar.msg
import std_msgs.msg

class MarkerImagePositionsResponse(genpy.Message):
  _md5sum = "3da72b51c07353b2973e59b88e43b41f"
  _type = "ar_track_service/MarkerImagePositionsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """ar_track_alvar/AlvarMarkers markers


================================================================================
MSG: ar_track_alvar/AlvarMarkers
Header header
AlvarMarker[] markers

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: ar_track_alvar/AlvarMarker
Header header
uint32 id
uint32 confidence
geometry_msgs/PoseStamped pose


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['markers']
  _slot_types = ['ar_track_alvar/AlvarMarkers']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       markers

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MarkerImagePositionsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.markers is None:
        self.markers = ar_track_alvar.msg.AlvarMarkers()
    else:
      self.markers = ar_track_alvar.msg.AlvarMarkers()

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
      _x = self
      buff.write(_struct_3I.pack(_x.markers.header.seq, _x.markers.header.stamp.secs, _x.markers.header.stamp.nsecs))
      _x = self.markers.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.markers.markers)
      buff.write(_struct_I.pack(length))
      for val1 in self.markers.markers:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.id, _x.confidence))
        _v3 = val1.pose
        _v4 = _v3.header
        buff.write(_struct_I.pack(_v4.seq))
        _v5 = _v4.stamp
        _x = _v5
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v4.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v6 = _v3.pose
        _v7 = _v6.position
        _x = _v7
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v8 = _v6.orientation
        _x = _v8
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.markers is None:
        self.markers = ar_track_alvar.msg.AlvarMarkers()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.markers.header.seq, _x.markers.header.stamp.secs, _x.markers.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.markers.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.markers.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.markers.markers = []
      for i in range(0, length):
        val1 = ar_track_alvar.msg.AlvarMarker()
        _v9 = val1.header
        start = end
        end += 4
        (_v9.seq,) = _struct_I.unpack(str[start:end])
        _v10 = _v9.stamp
        _x = _v10
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v9.frame_id = str[start:end].decode('utf-8')
        else:
          _v9.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.id, _x.confidence,) = _struct_2I.unpack(str[start:end])
        _v11 = val1.pose
        _v12 = _v11.header
        start = end
        end += 4
        (_v12.seq,) = _struct_I.unpack(str[start:end])
        _v13 = _v12.stamp
        _x = _v13
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v12.frame_id = str[start:end].decode('utf-8')
        else:
          _v12.frame_id = str[start:end]
        _v14 = _v11.pose
        _v15 = _v14.position
        _x = _v15
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v16 = _v14.orientation
        _x = _v16
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.markers.markers.append(val1)
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
      _x = self
      buff.write(_struct_3I.pack(_x.markers.header.seq, _x.markers.header.stamp.secs, _x.markers.header.stamp.nsecs))
      _x = self.markers.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.markers.markers)
      buff.write(_struct_I.pack(length))
      for val1 in self.markers.markers:
        _v17 = val1.header
        buff.write(_struct_I.pack(_v17.seq))
        _v18 = _v17.stamp
        _x = _v18
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v17.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.id, _x.confidence))
        _v19 = val1.pose
        _v20 = _v19.header
        buff.write(_struct_I.pack(_v20.seq))
        _v21 = _v20.stamp
        _x = _v21
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v20.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v22 = _v19.pose
        _v23 = _v22.position
        _x = _v23
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v24 = _v22.orientation
        _x = _v24
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.markers is None:
        self.markers = ar_track_alvar.msg.AlvarMarkers()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.markers.header.seq, _x.markers.header.stamp.secs, _x.markers.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.markers.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.markers.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.markers.markers = []
      for i in range(0, length):
        val1 = ar_track_alvar.msg.AlvarMarker()
        _v25 = val1.header
        start = end
        end += 4
        (_v25.seq,) = _struct_I.unpack(str[start:end])
        _v26 = _v25.stamp
        _x = _v26
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v25.frame_id = str[start:end].decode('utf-8')
        else:
          _v25.frame_id = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.id, _x.confidence,) = _struct_2I.unpack(str[start:end])
        _v27 = val1.pose
        _v28 = _v27.header
        start = end
        end += 4
        (_v28.seq,) = _struct_I.unpack(str[start:end])
        _v29 = _v28.stamp
        _x = _v29
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v28.frame_id = str[start:end].decode('utf-8')
        else:
          _v28.frame_id = str[start:end]
        _v30 = _v27.pose
        _v31 = _v30.position
        _x = _v31
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v32 = _v30.orientation
        _x = _v32
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.markers.markers.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4d = struct.Struct("<4d")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
class MarkerImagePositions(object):
  _type          = 'ar_track_service/MarkerImagePositions'
  _md5sum = '6f8847a33e8eeb46d46060969e9f10c3'
  _request_class  = MarkerImagePositionsRequest
  _response_class = MarkerImagePositionsResponse
