std_msgs/Header header
uint8 GPS_FIX_TYPE_NO_GPS = 0
uint8 GPS_FIX_TYPE_NO_FIX = 1
uint8 GPS_FIX_TYPE_2D_FIX = 2
uint8 GPS_FIX_TYPE_3D_FIX = 3
uint8 GPS_FIX_TYPE_DGPS = 4
uint8 GPS_FIX_TYPE_RTK_FLOATR = 5
uint8 GPS_FIX_TYPE_RTK_FIXEDR = 6
uint8 GPS_FIX_TYPE_STATIC = 7
uint8 GPS_FIX_TYPE_PPP = 8
uint8 fix_type
int32 lat
int32 lon
int32 alt
uint16 eph
uint16 epv
uint16 vel
uint16 cog
uint8 satellites_visible
int32 alt_ellipsoid
uint32 h_acc
uint32 v_acc
uint32 vel_acc
int32 hdg_acc
uint16 yaw
uint8 dgps_numch
uint32 dgps_age
