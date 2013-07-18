"""autogenerated by genpy from ar_track_service/MarkerPositionsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class MarkerPositionsRequest(genpy.Message):
  _md5sum = "3f71a696b0145f9b397021b0162b39fa"
  _type = "ar_track_service/MarkerPositionsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 pc

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

"""
  __slots__ = ['pc']
  _slot_types = ['sensor_msgs/PointCloud2']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pc

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MarkerPositionsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pc is None:
        self.pc = sensor_msgs.msg.PointCloud2()
    else:
      self.pc = sensor_msgs.msg.PointCloud2()

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
      buff.write(_struct_3I.pack(_x.pc.header.seq, _x.pc.header.stamp.secs, _x.pc.header.stamp.nsecs))
      _x = self.pc.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc.height, _x.pc.width))
      length = len(self.pc.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc.is_bigendian, _x.pc.point_step, _x.pc.row_step))
      _x = self.pc.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.pc.is_dense))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.pc is None:
        self.pc = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.pc.header.seq, _x.pc.header.stamp.secs, _x.pc.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc.height, _x.pc.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc.is_bigendian, _x.pc.point_step, _x.pc.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc.is_bigendian = bool(self.pc.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc.data = str[start:end].decode('utf-8')
      else:
        self.pc.data = str[start:end]
      start = end
      end += 1
      (self.pc.is_dense,) = _struct_B.unpack(str[start:end])
      self.pc.is_dense = bool(self.pc.is_dense)
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
      buff.write(_struct_3I.pack(_x.pc.header.seq, _x.pc.header.stamp.secs, _x.pc.header.stamp.nsecs))
      _x = self.pc.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc.height, _x.pc.width))
      length = len(self.pc.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc.is_bigendian, _x.pc.point_step, _x.pc.row_step))
      _x = self.pc.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.pc.is_dense))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.pc is None:
        self.pc = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.pc.header.seq, _x.pc.header.stamp.secs, _x.pc.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc.height, _x.pc.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc.is_bigendian, _x.pc.point_step, _x.pc.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc.is_bigendian = bool(self.pc.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc.data = str[start:end].decode('utf-8')
      else:
        self.pc.data = str[start:end]
      start = end
      end += 1
      (self.pc.is_dense,) = _struct_B.unpack(str[start:end])
      self.pc.is_dense = bool(self.pc.is_dense)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_2I = struct.Struct("<2I")
_struct_B2I = struct.Struct("<B2I")
"""autogenerated by genpy from ar_track_service/MarkerPositionsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import ar_track_alvar.msg
import std_msgs.msg

class MarkerPositionsResponse(genpy.Message):
  _md5sum = "3da72b51c07353b2973e59b88e43b41f"
  _type = "ar_track_service/MarkerPositionsResponse"
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
      super(MarkerPositionsResponse, self).__init__(*args, **kwds)
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
class MarkerPositions(object):
  _type          = 'ar_track_service/MarkerPositions'
  _md5sum = '32f0081587ad6045b911e5b3b4d0a21a'
  _request_class  = MarkerPositionsRequest
  _response_class = MarkerPositionsResponse