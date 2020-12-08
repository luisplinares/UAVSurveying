#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <apx15/RawUbx.h>
#include <apx15/GSOF.h>
#include <apx15/GNSS.h>
#include "serial/serial.h"
#include "comms.h"
#include "Utils.h"

//#include <limits>

#include <iostream>
#include<chrono>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

#include<fstream>
#include<iostream>
#include<list>
#include<string.h>
#include<iostream>
#include<unistd.h>
#include<fcntl.h>
#include<termios.h>
#include<sstream>
#include<iomanip>
#include <queue>
#include <thread>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#define DEFINE_BYTE_ARRAY_METHODS                                                                   \
	inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }           \
	inline uint8_t operator[](size_t i) const { return reinterpret_cast<const uint8_t *>(this)[i]; }

#define PACKED __attribute__((__packed__))

union PACKED UbxVal
{
	DEFINE_BYTE_ARRAY_METHODS
	uint8_t  val1;
	uint16_t val2;
	uint32_t val4;
	uint64_t val8;
};

struct PACKED UbxCfgKeyVal
{
	uint32_t key;
	UbxVal   val;
	uint8_t  len;
};

struct PACKED uBloxUbxConfig
{
	uint8_t hd1;
	uint8_t hd2;
	uint8_t classId;
	uint8_t msgId;
	uint16_t length;
	uint8_t version;
	uint8_t layers;
	uint8_t reserved1;
	uint8_t reserved2;
	// payload starts here at byte 10
};

union PACKED CfgMessage
{
	uBloxUbxConfig cfgValSet;
	uint8_t message[1000];
};

struct PACKED UbxNavHpPosLlh  // 0x01, 0x14
{
	uint8_t version;           ///< Message version (0 for this version)
	uint8_t reserved0[2];      ///< Reserved
	uint8_t flags;		       ///< Flags
	uint32_t iTOW;             ///< GPS time of week of the navigation epoch [ms]
	int32_t lon;               ///< Longitude [deg]
	int32_t lat;               ///< Latitude [deg]
	int32_t height;            ///< height above ellipsoid [mm]
	int32_t hMSL;              ///< height above mean sea level [mm]
	int8_t lonHp;              ///< High precision componant of lon [deg]. -99..+99. precise longitude = lon + (lonHp * 1e-2)
	int8_t latHp;              ///< High precision componant of lat [deg]. -99..+99. precise lattude = lat + (latHp * 1e-2)
	int8_t heightHp;           ///< High precision componant of height [mm]. -9..+9. precise height = height + (heightHp * 0.1)
	int8_t hMSLHp;             ///< High precision componant of hMSL [mm]. -9..+9. precise height above MSL = hMSL + (hMSLHp * 0.1)
	uint32_t hAccy;            ///< Horizontal accuracy estimate [mm]
	uint32_t vAccy;            ///< Vertical accuracy estimate [mm]
	uint8_t chk_a;
	uint8_t chk_b;
};

struct PACKED Ubx_Nav_RelPosNed
{
	uint8_t version;
	uint8_t reserved0;
	uint16_t refStationId;
	uint32_t iTOW;
	int32_t relPosN;
	int32_t relPosE;
	int32_t relPosD;
	int32_t relPosLength;
	int32_t relPosHeading;
	uint8_t reserved1[4];
	int8_t relPosHPN;
	int8_t relPosHPE;
	int8_t relPosHPD;
	int8_t relPosHPLength;
	uint32_t accN;
	uint32_t accE;
	uint32_t accD;
	uint32_t accLength;
	uint32_t accHeading;
	uint8_t reserved2[4];
	uint32_t flags;
	uint8_t chk_a;
	uint8_t chk_b;
};

struct PACKED Ubx_Nav_TimeGps
{
	uint32_t iTOW;
	int32_t fTOW;
	int16_t week;
	int8_t leapS;
	uint8_t valid;
	uint32_t tAcc;
	uint8_t chk_a;
	uint8_t chk_b;
};

struct PACKED Ubx_Nav_Pvt
{
	uint32_t iTOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	uint32_t tAcc;
	int32_t nano;
	uint8_t fixType;
	uint8_t fixStatusFlags;
	uint8_t flags2;
	uint8_t numSatellitesInNavSolution;
	int32_t lon;
	int32_t lat;
	int32_t height;
	int32_t hMSL;
	uint32_t hAcc;
	uint32_t vAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;
	int32_t groundSpeed;
	int32_t headMotion;
	uint32_t sAcc;
	uint32_t headAcc;
	uint16_t pDOP;
	uint8_t flags3;
	uint8_t reserved0[5];
	int32_t headVeh;
	int16_t magDec;
	uint16_t magAcc;
	uint8_t chk_a;
	uint8_t chk_b;
};

struct PACKED Meas
{
	double prMes;
	double cpMes;
	float doMes;
	uint8_t gnssId;
	uint8_t svId;
	uint8_t sigId;
	uint8_t freqId;
	uint16_t lockTime;
	uint8_t cno;
	uint8_t prStdev;
	uint8_t cpStdev;
	uint8_t doStdev;
	uint8_t trkStat;
	uint8_t reserved1;	
};

struct PACKED Ubx_Rxm_Rawx
{
	double rcvTow;//0
	uint16_t week;//8
	int8_t leapS;//10
	uint8_t numMeas;//11
	uint8_t recStat;//12
	uint8_t version;//13
	uint8_t reserved0[2];//14
	//Meas *meas;
};

union Ubx_Message
{
	UbxNavHpPosLlh ubxNavHpPosLlh;
	Ubx_Nav_RelPosNed ubx_Nav_RelPosNed;
	Ubx_Nav_TimeGps ubx_Nav_TimeGps;
	Ubx_Nav_Pvt ubx_Nav_Pvt;
	Ubx_Rxm_Rawx ubx_Rxm_Rawx;
};

struct PACKED Message_Reader
{
	uint8_t sync_a;
	uint8_t sync_b;
	uint8_t class_Id;
	uint8_t message_Id;
	uint16_t length;
	Ubx_Message ubx_Message;
};

VnSensor vs;
const int gnssBaudrate = 115200;

Ubx_Nav_Pvt ubx_Nav_Pvt;
Ubx_Nav_RelPosNed ubx_Nav_RelPosNed;
Ubx_Nav_TimeGps ubx_Nav_TimeGps;
UbxNavHpPosLlh ubxNavHpPosLlh;

int commsPort;

int64_t periodTime;
int64_t periodDuration;
int64_t fullPeriodDuration;
int countGnss = 0;

int countImu = 0;
int64_t imuPeriod;
int64_t imuTotalPeriod;
int64_t imuTotalLastTime;

uint32_t utc_offset_second;

#define APX_TIME_DEBUG 0

//ros::Publisher imu_pub;
ros::Publisher gsof_pub;
ros::Publisher rawUbx_pub;
ros::Publisher offsetFlag_pub;
ros::Publisher gnss_pub;

int calcOffsetFlag = 0;
int currentTimeSet = 0;

void recordOffsetTime(apx15::U32 gpsTimeInMilliSec)
{
  if (calcOffsetFlag == 0)
  {
    int32_t actualSec = gpsTimeInMilliSec/1000 - 18; //leapSecond;
    uint64_t actualNsec = (gpsTimeInMilliSec - actualSec*1000)*1000000;
	
	//Ubx_Nav_Pvt ubxNavPvt = ubx_Nav_Pvt;

	/*cout << "GNSS Status: " << (int)(ubxNavPvt.fixType) << endl;
	cout << "validDate: " << (ubxNavPvt.valid & 0b1) << endl;
	cout << "validTime: " << (ubxNavPvt.valid >> 1 & 0b1) << endl;
	cout << "fullyResolved: " << (ubxNavPvt.valid >> 2 & 0b1) << endl;
	cout << "confirmedDate: " << (ubxNavPvt.flags2 >> 6 & 0b1) << endl;
	cout << "confirmedTime: " << (ubxNavPvt.flags2 >> 7 & 0b1) << endl;*/
	
	if(currentTimeSet == 0){
		printf("\033[1m\033[31m GPS Time Not Valid!\033[0m\n");
		return;
	}

	/*tm gpsTime;
	gpsTime.tm_year = ubxNavPvt.year-1900;
	gpsTime.tm_mon = ubxNavPvt.month-1;
	gpsTime.tm_mday = ubxNavPvt.day;
	gpsTime.tm_hour = ubxNavPvt.hour;
	gpsTime.tm_min = ubxNavPvt.min;
	gpsTime.tm_sec = ubxNavPvt.sec;

	tm *pcGMTime = &gpsTime;
	time_t pcTime = timegm(pcGMTime);*/
    time_t pcTime = time(NULL);
    tm *pcGMTime = gmtime(&pcTime);
    if (APX_TIME_DEBUG)
    {
      printf("  pc: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
            pcGMTime->tm_year, pcGMTime->tm_mon, pcGMTime->tm_mday,
            pcGMTime->tm_wday, pcGMTime->tm_hour, pcGMTime->tm_min, pcGMTime->tm_sec);
    }

    time_t tTime = pcTime - actualSec + 40;
    tm *tGMTime = gmtime(&tTime);
    if (APX_TIME_DEBUG)
    {
      printf("temp: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
            tGMTime->tm_year, tGMTime->tm_mon, tGMTime->tm_mday,
            tGMTime->tm_wday, tGMTime->tm_hour, tGMTime->tm_min, tGMTime->tm_sec);
    } 

    /*cout << "actualSec: " << actualSec << endl;
    cout << "actualNsec: " << actualNsec << endl;
    cout << std::setprecision(20) << "tTime: " << tTime << endl;
    cout << std::setprecision(20) << "pcTime: " << pcTime << endl;*/

    if ((tGMTime->tm_wday == 0) && 
        (tGMTime->tm_hour == 0) && 
        (tGMTime->tm_min == 0))
    {
      tGMTime->tm_sec = 0;
      utc_offset_second = timegm(tGMTime);

      calcOffsetFlag = 1;

      printf("****** offset convert success! ******\n");
    }
    else
    {
      printf("\033[1m\033[31m time offset convert error!\033[0m\n");
      utc_offset_second = 0;
    }

    if (APX_TIME_DEBUG)
    {
      printf("week offset second:%d\n", utc_offset_second);
    }
  }

  offsetFlag_pub.publish(calcOffsetFlag);

}

void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	
	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
				COMMONGROUP_TIMEGPS | /*COMMONGROUP_YAWPITCHROLL |*/ COMMONGROUP_QUATERNION | COMMONGROUP_POSITION, // Note use of binary OR to configure flags.
				TIMEGROUP_GPSTOW,
				IMUGROUP_NONE,
				GPSGROUP_NONE,
				/*ATTITUDEGROUP_NONE*/ ATTITUDEGROUP_YPRU,
				INSGROUP_INSSTATUS | INSGROUP_POSU | INSGROUP_VELU, 
				GPSGROUP_NONE))
      // Not the type of binary packet we are expecting.
			return;

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the Packet structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
		uint64_t timeGps = p.extractUint64();
		//vec3f ypr = p.extractVec3f();
		vec4f qtn = p.extractVec4f();
		vec3d position = p.extractVec3d();
		uint64_t gpsTow = p.extractUint64();
		vec3f ypru = p.extractVec3f();
		uint16_t insStatus = p.extractUint16();
		float posU = p.extractFloat();
		float velU = p.extractFloat();

	    /*cout << "Binary Async TimeGps: " << timeGps << endl;
		//cout << std::setprecision(12) << "Binary Async YPR: " << ypr << endl;
		cout << std::setprecision(12) << "Binary Async QTN: " << qtn << endl;
		cout << std::setprecision(12) << "Binary Async Position: " << position.x << "; " << position.y << "; " << position.z << endl;
		printf("Binary Async InsStatus: %x \n", insStatus & 0b11 );
		printf("GNSSFix: %x \n", (insStatus >> 2) & 0b1 );
		printf("Error: %x \n", (insStatus >> 3) & 0b1111 );
		cout << "Binary Async Position Uncertainty: " << posU << endl;
		cout << "Binary Async Velocity Uncertainty: " << velU << endl;

		cout << "GNSS Fix: " << (int)ubx_Nav_Pvt.fixType << endl;

		cout << "pvt gnssFixOk: " << (ubx_Nav_Pvt.fixStatusFlags & 0b1) << endl;
		cout << "pvt diffSoln: " << ((ubx_Nav_Pvt.fixStatusFlags >> 1) & 0b1) << endl;
		cout << "pvt carrSoln: " << ((ubx_Nav_Pvt.fixStatusFlags >> 6) & 0b11) << endl;

		cout << "relposned gnssFixOk: " << (ubx_Nav_RelPosNed.flags & 0b1) << endl;
		cout << "relposned diffSoln: " << ((ubx_Nav_RelPosNed.flags >> 1) & 0b1) << endl;
		cout << "relposned relPosValid: " << ((ubx_Nav_RelPosNed.flags >> 2) & 0b1) << endl;
		cout << "relposned carrSoln: " << ((ubx_Nav_RelPosNed.flags >> 3) & 0b11) << endl;*/

    apx15::U32 gpsTimeInMilliSec = (apx15::U32)(gpsTow/1000000);
    recordOffsetTime( gpsTimeInMilliSec );

	if(calcOffsetFlag == 0){
		return;
	}

    apx15::GSOF gsof;
    
    //leap second is 18
    int32_t weekSec = gpsTimeInMilliSec/1000;
    int32_t utcSec = utc_offset_second + weekSec - 18;
    uint64_t utcNsec = (gpsTimeInMilliSec%1000)*1000000;

    /*time_t testTime = utcSec;
    tm *testGMTime = gmtime(&testTime);

      printf("  test: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
            testGMTime->tm_year, testGMTime->tm_mon, testGMTime->tm_mday,
            testGMTime->tm_wday, testGMTime->tm_hour, testGMTime->tm_min, testGMTime->tm_sec);*/

    
    gsof.sec  = utcSec;
    gsof.nsec = utcNsec;

	/*gsof.orientation = tf::createQuaternionMsgFromRollPitchYaw(ypr.z*DEGREE2PI,
																ypr.y*DEGREE2PI,
																ypr.x*DEGREE2PI);

	cout << "gsof.orientation.x: " << gsof.orientation.x << endl;
	cout << "gsof.orientation.y: " << gsof.orientation.y << endl;
	cout << "gsof.orientation.z: " << gsof.orientation.z << endl;
	cout << "gsof.orientation.w: " << gsof.orientation.w << endl;*/

	gsof.orientation.x = qtn.x;
	gsof.orientation.y = qtn.y;
	gsof.orientation.z = qtn.z;
	gsof.orientation.w = qtn.w;

	/*cout << "qtn.x: " << gsof.orientation.x << endl;
	cout << "qtn.y: " << gsof.orientation.y << endl;
	cout << "qtn.z: " << gsof.orientation.z << endl;
	cout << "qtn.w: " << gsof.orientation.w << endl;*/
    
    gsof.latitude = position.x;
    gsof.longitude = position.y;
    gsof.altitude = position.z;
    
	gsof.insstatus = insStatus & 0xff; //(insStatus & 0xff | 0b10) & 0b11111110; //prueba

	gsof.posu = posU;
	gsof.yawu = ypru.x;
	gsof.pitchu = ypru.y;
	gsof.rollu = ypru.z;

	if(gsof_pub.getNumSubscribers() > 0){
		gsof_pub.publish(gsof);
	}

    /*if (imu_pub.getNumSubscribers() > 0)
    {
      
      imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll*DEGREE2PI,
                                                                    pitch*DEGREE2PI,
                                                                    yaw*DEGREE2PI);

      imu_msg.orientation_covariance[0] = roll;
      imu_msg.orientation_covariance[1] = pitch;
      imu_msg.orientation_covariance[2] = yaw;
      imu_msg.orientation_covariance[3] = 0;//rollRMS;
      imu_msg.orientation_covariance[4] = 0;//pitchRMS;
      imu_msg.orientation_covariance[5] = 0;//yawRMS;

      imu_msg.angular_velocity.x = 0;//angularRate[0]*DEGREE2PI;
      imu_msg.angular_velocity.y = 0;//angularRate[1]*DEGREE2PI;
      imu_msg.angular_velocity.z = 0;//angularRate[2]*DEGREE2PI;

      imu_msg.linear_acceleration.x = 0;//acceleration[0];
      imu_msg.linear_acceleration.y = 0;//acceleration[1];
      imu_msg.linear_acceleration.z = 0;//acceleration[2];

      imu_pub.publish(imu_msg);
    }

    if (gnss_pub.getNumSubscribers() > 0)
    {
      navfix_msg.header.stamp.sec = imu_msg.header.stamp.sec;
      navfix_msg.header.stamp.nsec = imu_msg.header.stamp.nsec;

      navfix_msg.status.status  = GPS_Quality;
      navfix_msg.status.service = IMU_AlignmentStatus;

      navfix_msg.latitude  = latitude;
      navfix_msg.longitude = longitude;
      navfix_msg.altitude  = altitude;

      navfix_msg.position_covariance[0] = 0;//northPositionRMS;
      navfix_msg.position_covariance[1] = 0;//eastPositionRMS;
      navfix_msg.position_covariance[2] = 0;//downPositionRMS;

      gnss_pub.publish(navfix_msg);
    }*/

		/*cout << "Binary Async TimeGps: " << timeGps << endl;
		cout << "Binary Async YPR: " << ypr << endl;
		cout << "Binary Async GpsPosLla: " << gpsPosLla << endl;
		cout << "Binary Async Position: " << position << endl;
		printf("Binary Async InsStatus: %x \n", insStatus & 0b11 );
		printf("GNSSFix: %x \n", insStatus & 0b100 );
		printf("Error: %x \n", insStatus & 0b1111000 );
		//printf("Reserved1: %x \n", insStatus & 0b10000000 );
		//printf("Reserved8: %x \n", insStatus & 0b1111111100000000 );
		cout << "Binary Async Position Uncertainty: " << posU << endl;
		cout << "Binary Async Velocity Uncertainty: " << velU << endl;
		cout << " Num Satellites: " << (uint16_t)ubx_Nav_Pvt.numSatellitesInNavSolution << endl;
		printf("Fix Type: %x \n", ubx_Nav_Pvt.fixType );
		printf("GNSS Time %ld \n", periodDuration);

		countImu++;
		
		if(countImu == 400){
			int64_t newImuTime = std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch()
			).count();
			countImu = 0;
			imuTotalPeriod = newImuTime - imuTotalLastTime;
			imuTotalLastTime = newImuTime;
		}
		printf("IMU Total Period %ld \n", imuTotalPeriod);*/
	}
}

void uBloxFletcher8 ( unsigned char c, unsigned char *cka, unsigned char *ckb )
{
	*cka += c;
	*ckb += *cka;
}

int setupComAttrs ( int fd, int speed )
{
	struct termios options;
	memset ( &options, 0, sizeof options );

	if ( tcgetattr ( fd, &options ) != 0 )
	{
		//Log::msg ( "uBloxPosition::SetInterfaceAttr:error %d from tcgetattr\n", errno );
		return -1;
	}

	cfsetospeed ( &options, speed );
	cfsetispeed ( &options, speed );

	options.c_cflag |= ( CLOCAL | CREAD ); // ignore modem controls,
	// enable reading
	// No parity (8N1):
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Disable hardware flow control:
	options.c_cflag &= ~CRTSCTS;

	// Raw input
	options.c_lflag &= ~ ( ICANON | ECHO | ECHOE | ISIG );

	// Disable software flow control
	options.c_iflag &= ~ ( IXON | IXOFF | IXANY | ICRNL );

	// Raw output
	options.c_oflag &= ~OPOST;

	// Read Timeouts
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 10;

	if ( tcsetattr ( fd, TCSANOW, &options ) != 0 )
	{
		//Log::msg ( "uBloxPosition::SetInterfaceAttr:error %d from tcsetattr\n", errno );
		return -1;
	}

	return 0;
}

void setComBlocking ( int fd, int should_block )
{
	struct termios options;
	memset ( &options, 0, sizeof options );

	if ( tcgetattr ( fd, &options ) != 0 )
	{
		//Log::msg ( "error %d from tggetattr\n", errno );
		return;
	}

	options.c_cc[VMIN]  = should_block ? 1 : 0;
	options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if ( tcsetattr ( fd, TCSANOW, &options ) != 0 )
		std::cout << "error setting term attributes." << std::endl;
		//Log::msg ( "error %d setting term attributes\n", errno );
}

void setupComPort ( const char * portname )
{
	//Log::msg ( "uBloxPosition::SetupComPort:Opening COM Port %s at 460800\n", portname );

	//commsPort = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	commsPort = open ( portname, O_RDWR | O_NOCTTY );

	//commsPort = open (portname, O_RDWR | O_NOCTTY | O_NDELAY);
	if ( commsPort < 0 )
	{
		//Log::msg ( "uBloxPosition::SetupComPort:error %d opening %s: %s\n", errno, portname, strerror ( errno ) );
		throw std::exception();
		return;
	}

	//El-Baudrate
	setupComAttrs ( commsPort, B115200 ); // set speed to 460800 bps, 8n1 (no parity)
	setComBlocking ( commsPort, 0 );    // set no blocking
	//WaitForReady();
	//sleep(2);
}

void sendUbxConfig ( std::list<UbxCfgKeyVal> keyVal, uint8_t dest )
{
	CfgMessage cfgMessasge;

//	Log::msg("sendUbxConfig- keyVal len:%d\n",keyVal.size());

	memset ( cfgMessasge.message, 0, sizeof ( cfgMessasge.message ) );
	unsigned char cka = 0, ckb = 0;

	cfgMessasge.cfgValSet.hd1 = 0xB5;
	cfgMessasge.cfgValSet.hd2 = 0x62;
	cfgMessasge.cfgValSet.classId = 0x06;
	cfgMessasge.cfgValSet.msgId = 0x8A;          // UBX-CFG-VALSET
	// length
	cfgMessasge.cfgValSet.version = 0x01;
	cfgMessasge.cfgValSet.layers = dest; // Flash(4) + BBR(2) + RAM(1)

	cfgMessasge.cfgValSet.reserved1 = 0x00;
	cfgMessasge.cfgValSet.reserved2 = 0x00;

	int i, lenPayload = 4;

	std::list<UbxCfgKeyVal>::iterator it;

	it = keyVal.begin();

	i = 10;
	while ( it != keyVal.end() )
	{
		memcpy ( &cfgMessasge.message[i], &it->key, it->len );
		lenPayload += it->len;
		i += it->len;
		it++;
//		Log::msg("sendUbxConfig- lenPayload:%d\n",lenPayload);
	}

	cfgMessasge.cfgValSet.length = lenPayload;

	for ( i = 2; i <= lenPayload + 4 + 1; i++ ) // + 4 for class/id/length
		uBloxFletcher8 ( cfgMessasge.message[i], &cka, &ckb );

	cfgMessasge.message[i] = cka;
	i++;
	cfgMessasge.message[i] = ckb;


//	Log::msg("sendUbxConfig- write len:%d\n", lenPayload+2+4+2);
	// send message
	/*int n = */write ( commsPort, cfgMessasge.message, lenPayload + 2 + 4 + 2 ); // 2 for header + 4 for class/id/length + 2 for cka/ckb
	//Log::msg ( "sendUbxConfig- write len:%d\n", n );

	return;
}

void sendUbloxSetup ()
{
	//Log::msg ( "sendUbloxSetup-\n" );
	std::list<UbxCfgKeyVal> keyVals;

	UbxCfgKeyVal keyVal;

	// UART1

	/*keyVal.key = 0x10520005; // UART1 ENABLE
	keyVal.val.val4 = 1; //
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10640006; // SPI DISABLE
	keyVal.val.val4 = 0; //
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10730001; // CFG-UART1INPROT-UBX
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10730002; // CFG-UART1INPROT-NMEA
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10730004; // CFG-UART1INPROT-RTCM3X
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10740001; // CFG-UART1OUTPROT-UBX
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10740002; // CFG-UART1OUTPROT-NMEA
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );*/

	//NMEA

	keyVal.key = 0x209100cb; //GLL Rate
	keyVal.val.val1 = 0;
	keyVal.len = 5;
	keyVals.push_back(keyVal);
	
	keyVal.key = 0x209100c6; //GSV Rate
	keyVal.val.val1 = 0;
	keyVal.len = 5;
	keyVals.push_back(keyVal);

	keyVal.key = 0x209100c1; //GSA Rate
	keyVal.val.val1 = 0;
	keyVal.len = 5;
	keyVals.push_back(keyVal);

	keyVal.key = 0x209100bc; //GGA Rate
	keyVal.val.val1 = 0;
	keyVal.len = 5;
	keyVals.push_back(keyVal);

	keyVal.key = 0x209100b2; //VTG Rate
	keyVal.val.val1 = 0;
	keyVal.len = 5;
	keyVals.push_back(keyVal);

	keyVal.key = 0x209100ad; //RMC Rate
	keyVal.val.val1 = 5;
	keyVal.len = 5;
	keyVals.push_back(keyVal);


	/*keyVal.key = 0x10740004; // CFG-UART1OUTPROT-RTCM3X
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x40520001; // CFG-UART1-BAUDRATE 0x40520001
	keyVal.val.val4 = 115200; //El-Baudrate
	keyVal.len = 8; // key(4) + val(4)
	keyVals.push_back ( keyVal );*/

//  UART2

	keyVal.key = 0x10750001; // CFG-UART2INPROT-UBX
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10750002; // CFG-UART2INPROT-NMEA
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10750004; // CFG-UART2INPROT-RTCM3X
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10760001; // CFG-UART2OUTPROT-UBX
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10760002; // CFG-UART2OUTPROT-NMEA
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10760004; // CFG-UART2OUTPROT-RTCM3X
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x40530001; // CFG-UART2-BAUDRATE
	keyVal.val.val4 = 115200;  //El-Baudrate
	keyVal.len = 8; // key(4) + val(4)
	keyVals.push_back ( keyVal );

	// Other

	keyVal.key = 0x2091004a; //UBX-NAV-TIMEGPS
	keyVal.val.val1 = 1;
	keyVal.len = 5;
	keyVals.push_back(keyVal);

	keyVal.key = 0x20910036; // UBXMSGNAV_HPPOSLLH to USB
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x20910090; //UBX-NAV-RELPOSNED 0x20910090
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x20910009 ; // UBXMSGNAV_PVT to USB
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x209102a7; // UBX_RXM_RAWX
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );


	keyVal.key = 0x2091008b; // UBXMSGNAV_SVIN to USB
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x30210001; // Nav Rate
	keyVal.val.val2 = 200;
	keyVal.len = 6; // key(4) + val(2)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10780001; // CFG-USBOUTPROT-USB
	keyVal.val.val1 = 1;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10780002; // CFG-USBOUTPROT-NMEA
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	keyVal.key = 0x10780004; // CFG-USBOUTPROT-RTCM3X
	keyVal.val.val1 = 0;
	keyVal.len = 5; // key(4) + val(1)
	keyVals.push_back ( keyVal );

	sendUbxConfig ( keyVals, 0x1 );	 // Flash(4) + BBR(2) + RAM(1)
}

void printMessage(Message_Reader message_Reader)
{
	if(message_Reader.class_Id == 0x01 && message_Reader.message_Id == 0x3c)
	{
		/*printf("UBX-NAV-RELPOSNED: \n");
		printf("sync_a: %x , ", message_Reader.sync_a);
		printf("sync_b: %x , ", message_Reader.sync_b);
		printf("class_Id: %x , ", message_Reader.class_Id);
		printf("message_Id: %x , ", message_Reader.message_Id);
		printf("length: %d \n", message_Reader.length);*/
		
		ubx_Nav_RelPosNed = message_Reader.ubx_Message.ubx_Nav_RelPosNed;
		
		/*std::cout << "ubx_Nav_Pvt.iTOW: " << ubx_Nav_Pvt.iTOW;
		std::cout << "  ubx_Nav_RelPosNed.iTOW: " << ubx_Nav_RelPosNed.iTOW;
		std::cout << "  ubx_Nav_TimeGps.iTOW: " << ubx_Nav_TimeGps.iTOW;
		std::cout << "  ubxNavHpPosLlh.iTOW: " << ubxNavHpPosLlh.iTOW << std::endl;*/

		

		/*std::cout << ubx_Nav_RelPosNed.accN << std::endl;
		std::cout << ubx_Nav_RelPosNed.accE << std::endl;
		std::cout << ubx_Nav_RelPosNed.accD << std::endl;

		std::cout << ubx_Nav_Pvt.hAcc << std::endl;
		std::cout << ubx_Nav_Pvt.vAcc << std::endl;

		std::cout << PosAccX << std::endl;
		std::cout << PosAccY << std::endl;
		std::cout << PosAccZ << std::endl;*/
		
		
		
		/*ostringstream oss1;
		oss1 << "$VNGPS,58," << ubx_Nav_TimeGps.iTOW << "," << ubx_Nav_TimeGps.week
			<< "," << (uint16_t)ubx_Nav_Pvt.fixType << "," << (uint16_t)ubx_Nav_Pvt.numSatellitesInNavSolution
			<< "," << Lat << "," << Long << "," << Alt 
			<< "," << 0 << "," << 0 << "," << 0 << "," << 0
			//<< "," << AccN+.1 << "," << AccE+.1 << "," << AccD+.1 
			<< "," << PosAccY << "," << PosAccZ
			<< "," << SAcc << "," << ubx_Nav_Pvt.tAcc;*/
		
	}
	else if(message_Reader.class_Id == 0x01 && message_Reader.message_Id == 0x14){
		/*printf("UBX-NAV-HPPOSLLH: \n");
		printf("sync_a: %x , ", message_Reader.sync_a);
		printf("sync_b: %x , ", message_Reader.sync_b);
		printf("class_Id: %x , ", message_Reader.class_Id);
		printf("message_Id: %x , ", message_Reader.message_Id);
		printf("length: %d \n", message_Reader.length);*/
		ubxNavHpPosLlh = message_Reader.ubx_Message.ubxNavHpPosLlh;
		
	}
	else if(message_Reader.class_Id == 0x01 && message_Reader.message_Id == 0x20){
		/*printf("UBX-NAV-TIMEGPS: \n");
		printf("sync_a: %x , ", message_Reader.sync_a);
		printf("sync_b: %x , ", message_Reader.sync_b);
		printf("class_Id: %x , ", message_Reader.class_Id);
		printf("message_Id: %x , ", message_Reader.message_Id);
		printf("length: %d \n", message_Reader.length);*/
		ubx_Nav_TimeGps = message_Reader.ubx_Message.ubx_Nav_TimeGps;
	}
	else if(message_Reader.class_Id == 0x01 && message_Reader.message_Id == 0x07)
	{
		/*printf("UBX-NAV-PVT: \n");
		printf("sync_a: %x , ", message_Reader.sync_a);
		printf("sync_b: %x , ", message_Reader.sync_b);
		printf("class_Id: %x , ", message_Reader.class_Id);
		printf("message_Id: %x , ", message_Reader.message_Id);
		printf("length: %d \n", message_Reader.length);*/
		ubx_Nav_Pvt = message_Reader.ubx_Message.ubx_Nav_Pvt;
		
		/*std::cout << "ubx_Nav_Pvt: " << message_Reader.ubx_Message.ubx_Nav_Pvt.fixType 
			<< "," << message_Reader.ubx_Message.ubx_Nav_Pvt.numSatellitesInNavSolution
			<< std::endl;*/
	}
	/*else if (message_Reader.class_Id == 0x05 && message_Reader.message_Id == 0x01)
	{
		printf("UBX-ACK-ACK: \n");
		printf("sync_a: %x , ", message_Reader.sync_a);
		printf("sync_b: %x , ", message_Reader.sync_b);
		printf("class_Id: %x , ", message_Reader.class_Id);
		printf("message_Id: %x , ", message_Reader.message_Id);
		printf("length: %d \n", message_Reader.length);
	}*/
	if(ubx_Nav_Pvt.iTOW != 0 && ubx_Nav_Pvt.iTOW == ubx_Nav_RelPosNed.iTOW &&
		ubx_Nav_Pvt.iTOW == ubx_Nav_TimeGps.iTOW && 
		ubx_Nav_Pvt.iTOW == ubxNavHpPosLlh.iTOW){
		
		double Lat = ( double ) ubxNavHpPosLlh.lat * 1e-7 + ( double ) ubxNavHpPosLlh.latHp * 1e-9;
		double Long = ( double ) ubxNavHpPosLlh.lon * 1e-7 + ( double ) ubxNavHpPosLlh.lonHp * 1e-9;
		double Alt = ( ( double ) ubxNavHpPosLlh.height + ( double ) ubxNavHpPosLlh.heightHp * 0.1 ) / 1000.0;

		double VelN = ( ( double ) ubx_Nav_Pvt.velN ) / 1000.0;
		double VelE = ( ( double ) ubx_Nav_Pvt.velE ) / 1000.0;
		double VelD = ( ( double ) ubx_Nav_Pvt.velD ) / 1000.0;

		double SAcc = ( (double) ubx_Nav_Pvt.sAcc ) / 1000.0;

		double AccN = ( ( double ) ubx_Nav_RelPosNed.accN ) * 0.1 / 1000.0;
		double AccE = ( ( double ) ubx_Nav_RelPosNed.accE ) * 0.1 / 1000.0;
		double AccD = ( ( double ) ubx_Nav_RelPosNed.accD ) * 0.1 / 1000.0;

		double PosAccX = (double)ubx_Nav_Pvt.hAcc / 1000.0;
		double PosAccY = PosAccX;
		double PosAccZ = (double)ubx_Nav_Pvt.vAcc / 1000.0;

		double tow = (double)ubx_Nav_TimeGps.iTOW / 1000.0;

		int pvtCarrSoln = ((ubx_Nav_Pvt.fixStatusFlags >> 6) & 0b11);

		int relposnedDiffSoln = ((ubx_Nav_RelPosNed.flags >> 1) & 0b1);
		int relposnedRelPosValid = ((ubx_Nav_RelPosNed.flags >> 2) & 0b1);
		int relposnedCarrSoln = ((ubx_Nav_RelPosNed.flags >> 3) & 0b11);

		cout << "pvt fix type: " << (uint16_t)ubx_Nav_Pvt.fixType << endl;

		cout << "pvt carrSoln: " << pvtCarrSoln << endl;

		cout << "relposned diffSoln: " << relposnedDiffSoln << endl;
		cout << "relposned relPosValid: " << relposnedRelPosValid << endl;
		cout << "relposned carrSoln: " << relposnedCarrSoln << endl;

		int gnssFixOk = relposnedDiffSoln == 1 && relposnedRelPosValid == 1 && relposnedCarrSoln == 2 && ubx_Nav_Pvt.fixType == 3 && pvtCarrSoln == 2 ? 3 : 0;
		gnssFixOk = ubx_Nav_Pvt.fixType;  //prueba
		ostringstream oss;
		//$VNWRG,58, //$VNWRG,59,
		oss << "$VNWRG,58," << std::setprecision(12)
			<< tow << "," << ubx_Nav_TimeGps.week
			<< "," << gnssFixOk << "," << (uint16_t)ubx_Nav_Pvt.numSatellitesInNavSolution
			<< "," << Lat << "," << Long << "," << Alt
			<< "," << VelN << "," << VelE << "," << VelD
			//<< "," << AccN << "," << AccE << "," << AccD
			<< "," << PosAccX << "," << PosAccY << "," << PosAccZ
			<< "," << SAcc << "," << ubx_Nav_Pvt.tAcc;

		/*oss << "$VNWRG,58," << std::setprecision(12)
			<< tow << "," << ubx_Nav_TimeGps.week
			<< "," << gnssFixOk << "," << (uint16_t)ubx_Nav_Pvt.numSatellitesInNavSolution
			<< "," << 11.0 << "," << -74.0 << "," << 10.0
			<< "," << 0.0 << "," << 0.0 << "," << 0.0
			<< "," << 0.001 << "," << 0.001 << "," << 0.001
			<< "," << 0.001 << "," << ubx_Nav_Pvt.tAcc;*/

		std::__cxx11::string gpsAiding = oss.str();

		apx15::GNSS gnss;
		gnss.fix = ubx_Nav_Pvt.fixType;
		gnss.sat = ubx_Nav_Pvt.numSatellitesInNavSolution;
		gnss.rtk = relposnedCarrSoln;
		gnss_pub.publish(gnss);

		//std::cout << gpsAiding << std::endl;

		/*std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
			std::chrono::system_clock::now().time_since_epoch()
		);*/

		try{
		      std::cout << 
		      vs.transaction( gpsAiding )//; 
		      << std::endl;
		}
		catch(exception& ex){
			cout << ex.what() << endl;
		}
		
		cout << "Fecha: " << ubx_Nav_Pvt.year << "/" << (int)ubx_Nav_Pvt.month
			<< "/" << (int)ubx_Nav_Pvt.day << " " << (int)ubx_Nav_Pvt.hour << ":" 
			<< (int)ubx_Nav_Pvt.min << ":" << (int)ubx_Nav_Pvt.sec << endl;

		time_t pcTime = time(NULL);
		tm *pcGMTime = gmtime(&pcTime);
		printf("Fecha  pc: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
				pcGMTime->tm_year+1900, pcGMTime->tm_mon+1, pcGMTime->tm_mday,
				pcGMTime->tm_wday, pcGMTime->tm_hour, pcGMTime->tm_min, pcGMTime->tm_sec);
		cout << "pcTime: " << pcTime << endl;

		tm gpsTime;
		gpsTime.tm_year = ubx_Nav_Pvt.year-1900;
		gpsTime.tm_mon = ubx_Nav_Pvt.month-1;
		gpsTime.tm_mday = ubx_Nav_Pvt.day;
		gpsTime.tm_hour = ubx_Nav_Pvt.hour;
		gpsTime.tm_min = ubx_Nav_Pvt.min;
		gpsTime.tm_sec = ubx_Nav_Pvt.sec;
		cout << "gpsTime: " << timegm(&gpsTime) << endl;
		
		Ubx_Nav_Pvt ubxNavPvt = ubx_Nav_Pvt;

		if( currentTimeSet == 0 &&
			(ubxNavPvt.flags2 & 0b11000000) == 0b11000000 &&
			(ubxNavPvt.valid & 0b111) == 0b111 &&
			ubxNavPvt.fixType == 3){
			
			stringstream ss;
			ss << "\"" << (ubxNavPvt.year) << "/"
			<< ((int)ubxNavPvt.month) << "/"
			<< ((int)ubxNavPvt.day) << " "
			<< ((int)ubxNavPvt.hour) << ":"
			<< ((int)ubxNavPvt.min) << ":"
			<< ((int)ubxNavPvt.sec) << "\"";

			cout << "Fecha: " << ss.str() << endl;
			string result = exec ( "%s %s", "/home/srcc/Documents/Livox/ws_livox/setdate.sh", ss.str().c_str() );
			char c_result[result.length()];
			strcpy(c_result, result.c_str());
			if(strstr(c_result, "EXITO")){
				cout << "System DateTime Successfully Updated!" << endl;
				currentTimeSet = 1;
			}else{
				cout << "System DateTime COULD NOT BE Updated!" << endl;
			}
		}
		/*cout << "GNSS Status: " << (int)(ubxNavPvt.fixType) << endl;
		cout << "validDate: " << (ubxNavPvt.valid & 0b1) << endl;
		cout << "validTime: " << (ubxNavPvt.valid >> 1 & 0b1) << endl;
		cout << "fullyResolved: " << (ubxNavPvt.valid >> 2 & 0b1) << endl;
		cout << "confirmedDate: " << (ubxNavPvt.flags2 >> 6 & 0b1) << endl;
		cout << "confirmedTime: " << (ubxNavPvt.flags2 >> 7 & 0b1) << endl;
		*/

		/*cout << ubx_Nav_Pvt.iTOW << " " << ubx_Nav_RelPosNed.iTOW 
			<< " " << ubx_Nav_TimeGps.iTOW << " " << ubxNavHpPosLlh.iTOW << endl;

		cout << (ubx_Nav_Pvt.iTOW^ubx_Nav_RelPosNed.iTOW)
			<< " " << (ubx_Nav_Pvt.iTOW^ubx_Nav_TimeGps.iTOW) << " " 
			<< (ubx_Nav_Pvt.iTOW^ubxNavHpPosLlh.iTOW) << endl;*/
			
		/*cout << "GNSS Period: " << ms.count()-periodTime << endl;
		periodTime = ms.count();
		countGnss++;
		if(countGnss == 5){
			countGnss = 0;
			fullPeriodDuration = ms.count() - periodDuration;
			periodDuration = ms.count();
		}
		cout << "GNSS Full Period: " << fullPeriodDuration << endl;*/
	}
}

string SensorPort;
void findPorts()
{
	string result = exec ( "/home/srcc/Documents/Livox/ws_livox/checkDev.sh" );
	
	char buffer[result.length()];
	strcpy(buffer, result.c_str());
		
	char *deviceArray[30];
	char *deviceParts[5];
	
	bool vnPortSet = false;
	int deviceCnt = mkarray ( deviceArray, buffer, "\n" );
	for (int i=0;i<deviceCnt;i++)
	{
		printf("%s \n",deviceArray[i]);
		int partCnt = mkarray ( deviceParts, deviceArray[i], " " );
		
		if (strcmp(deviceParts[2],"FTDI_FT232R_USB_UART_A50285BI") == 0)
		{
            if(strcmp(deviceParts[0], "/dev/ttyUSB0") == 0)
			{
				/*cout << "Reading port /dev/ttyUSB1: " << endl; 
				FILE *file;

				file = fopen("/dev/ttyUSB1", "r");
				char buff[200];
				fread(buff, sizeof(char), 200, file);
				fclose(file);
				if(strstr(buff, "GNRMC"))
				{*/
					SensorPort = "/dev/ttyUSB2";
					vnPortSet = true;
				/*}
				else{
					ros::param::set("/TimeSyncPort", "/dev/ttyUSB2");
					SensorPort = "/dev/ttyUSB1";
				}*/
			}
			else if(strcmp(deviceParts[0], "/dev/ttyUSB1") == 0)
			{
				/*cout << "Reading port /dev/ttyUSB0: " << endl; 
				FILE *file;

				file = fopen("/dev/ttyUSB0", "r");
				char buff[200];
				fread(buff, sizeof(char), 200, file);
				fclose(file);
				if(strstr(buff, "GNRMC")){*/
					SensorPort = "/dev/ttyUSB2";
					vnPortSet = true;
				/*}
				else{
					ros::param::set("/TimeSyncPort", "/dev/ttyUSB2");
					SensorPort = "/dev/ttyUSB0";
				}*/
			}
			else if(strcmp(deviceParts[0], "/dev/ttyUSB2") == 0)
			{
				/*cout << "Reading port /dev/ttyUSB0: " << endl; 
				FILE *file;

				file = fopen("/dev/ttyUSB0", "r");
				char buff[200];
				fread(buff, sizeof(char), 200, file);
				fclose(file);
				if(strstr(buff, "GNRMC")){*/
					SensorPort = "/dev/ttyUSB1";
					vnPortSet = true;
				/*}
				else{
					ros::param::set("/TimeSyncPort", "/dev/ttyUSB1");
					SensorPort = "/dev/ttyUSB0";	
				}*/
			}
		}
	}
	if(vnPortSet == false){
		SensorPort = "/dev/ttyUSB1";
		printf("findPorts- Found SensorPort on %s\n",SensorPort.c_str());
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apx15_gsof_driver");

  ros::NodeHandle gnss_inertial_nh("gnss_inertial"), private_nh("~");

  std::string vnport;
  ros::param::get("~vnport",vnport);

  //imu_pub  = gnss_inertial_nh.advertise<sensor_msgs::Imu>("imu", 1, false);
  gsof_pub = gnss_inertial_nh.advertise<apx15::GSOF>("gsof", 1, false);
  rawUbx_pub = gnss_inertial_nh.advertise<apx15::RawUbx>("rawubx", 1, false);
  offsetFlag_pub = gnss_inertial_nh.advertise<std_msgs::Int32>("offsetflag", 1, false);
  gnss_pub = gnss_inertial_nh.advertise<apx15::GNSS>("gnss_info", 1, false);

	const char* zedPort = "/dev/ttyACM0";

	//VnSensor
	//const string SensorPort = vnport;
	findPorts();

	const uint32_t SensorBaudrate = 921600; //115200 //921600

	cout << "SensorPort: " << SensorPort << endl;

	vs.connect(SensorPort, SensorBaudrate);
	
	//vs.setResponseTimeoutMs(10000);
	//vs.reset(true);
	//sleep(10);
	cout << "connected" << endl;
	//vs.changeBaudRate(921600);//current serial port
	//cout << "changed baud rate" << endl;
	
	//vs.writeGpsConfiguration(GPSMODE_EXTERNALGPS, PPSSOURCE_SYNCINRISING);
	//cout << "changed gps mode external" << endl;
	//return 0;
	//vs.writeSerialBaudRate(uint32_t(921600), uint8_t(1));
	//cout << "changed baudrate port 1" << endl;
	//vs.writeAsyncDataOutputType(VNOFF, uint8_t(2));
	//cout << "changed data output OFF port 2" << endl;
	//vs.writeAsyncDataOutputType(VNOFF, uint8_t(1));
	//cout << "changed data output INS port 1" << endl;
	
	BinaryOutputRegister bor (
		ASYNCMODE_PORT1,
		2, // 400hz
		COMMONGROUP_TIMEGPS | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_POSITION, // Note use of binary OR to configure flags.
		TIMEGROUP_GPSTOW,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_INSSTATUS | INSGROUP_POSU | INSGROUP_VELU,
		GPSGROUP_NONE);
	BinaryOutputRegister bor2 (
		ASYNCMODE_PORT2,
		2, // 400hz
		COMMONGROUP_TIMEGPS | /*COMMONGROUP_YAWPITCHROLL |*/ COMMONGROUP_QUATERNION | COMMONGROUP_POSITION, // Note use of binary OR to configure flags.
		TIMEGROUP_GPSTOW,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_YPRU,
		INSGROUP_INSSTATUS | INSGROUP_POSU | INSGROUP_VELU,
		GPSGROUP_NONE);
	
	BinaryOutputRegister borNothing (
		ASYNCMODE_PORT1,
		2, // 400hz
		COMMONGROUP_NONE, // Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE);
	BinaryOutputRegister borNothing2 (
		ASYNCMODE_PORT2,
		2, // 400hz
		COMMONGROUP_NONE, // Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE);
	
	//vs.writeBinaryOutput1(bor);
	//vs.writeBinaryOutput2(borNothing);
	//vs.writeBinaryOutput3(borNothing);

	//vs.writeBinaryOutput1(bor2);
	//vs.writeBinaryOutput2(borNothing2);
	//vs.writeBinaryOutput3(borNothing2);
	
	//vs.writeSettings();
	//cout << "Settings written to non-volatile memory" << endl;
	//return 0;

	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);
	
	setupComPort(zedPort);
	sendUbloxSetup();

	char readBuffer[5000];
	char message[3000];
	uint16_t *length;
	uint16_t payloadLength = 0;
	int status = 0;
	int count = 0;
	int payloadCount = 0;
	char sync_a = 0xb5;
	char sync_b = 0x62;

	periodTime = std::chrono::duration_cast< std::chrono::milliseconds >(
			std::chrono::system_clock::now().time_since_epoch()
		).count();
	imuPeriod = periodTime;
	imuTotalLastTime = periodTime;
	periodDuration = periodTime;

	int readBytes;

	//int enviados = 0;
	while(ros::ok())
	{
		readBytes = read(commsPort, readBuffer, sizeof(readBuffer));
		if(readBytes != -1 && readBytes != 0)
		{
			for(int i = 0; i<readBytes; i++)
			{
				char readByte = readBuffer[i];
				if(status == 0 && readByte == sync_a)
				{
					status = 1;
				}
				else if (status == 1 && readByte == sync_b)
				{
					message[count++] = sync_a;
					message[count++] = sync_b;
					status = 2;
				}
				else if(status == 2)
				{
					message[count++] = readByte;
					status = 3;
				}
				else if (status == 3)
				{
					message[count++] = readByte;
					status = 4;
				}
				else if (status == 4)
				{
					status = 5;
					message[count++] = readByte;
				}
				else if(status == 5)
				{
					status = 6;
					message[count++] = readByte;
					length = (uint16_t *)&message[count-2];
					payloadLength = *length;
				}else if(status == 6)
				{
					message[count++] = readByte;
					payloadCount++;
					if(payloadCount == payloadLength)
					{
						status = 7;
						payloadLength = 0;
						payloadCount = 0;
						length = nullptr;
					}
				}else if(status == 7)
				{
					message[count++] = readByte;
					status = 8;
				}
				else if(status == 8)
				{
					message[count++] = readByte;
					/*for(int i = 0; i <= count; i++)
					{
						printf("%x ", (uint8_t)message[i]);
					}
					printf("\n");*/

					Message_Reader *message_Reader = (Message_Reader *)message;
					if(message_Reader->class_Id == 0x02 && message_Reader->message_Id == 0x15) //UBX-RXM-RAWX
					{
						
						/*Ubx_Rxm_Rawx_Queued ubx_Rxm_Rawx_Queued;
						//ubx_Rxm_Rawx_Queued.message = (char *)malloc(sizeof(char)*count);
						memcpy(ubx_Rxm_Rawx_Queued.message, message, count);
						ubx_Rxm_Rawx_Queued.len = count;
						
						//ubx_Rxm_Rawx_Queue.push(ubx_Rxm_Rawx_Queued);
						ubx_Rxm_Rawx_Queue.emplace(ubx_Rxm_Rawx_Queued);*/

						if (rawUbx_pub.getNumSubscribers() > 0)
						{
							apx15::RawUbx rawUbx_msg;
							std::vector<uint8_t> v(message, message+count);
							rawUbx_msg.ubxRawData = v;
							rawUbx_pub.publish(rawUbx_msg);
							//cout << "enviados: " << ++enviados << endl;
						}

					}
					else
					{
						printMessage(*message_Reader);
					}
					//memset(message, 0, 1000);
					count = 0;
					status = 0;
				}
			}
		}
	}

	close(commsPort);
	
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();	
	cout << "Se acabó!" <<endl;
	return 0;
}


