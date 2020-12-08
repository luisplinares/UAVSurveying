#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <livox_mapping/CustomMsg.h>
#include <livox_mapping/RawUbx.h>
#include <livox_mapping/GSOF.h>
#include <livox_mapping/GNSS.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <pcl/common/transforms.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <map>
#include <pthread.h>
#include <queue>
#include <thread>
#include <chrono>
#include <iomanip>
#include <locale>

#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <termios.h>

#include <bits/stdc++.h>
#include <sys/stat.h> 
#include <sys/types.h> 

#include <rosbag/bag.h>

#include "Utils.h"

typedef pcl::PointXYZI PointType;
using namespace std;

template <class charT, charT sep>
class punct_facet: public std::numpunct<charT>
{
	protected:
		charT do_decimal_point() const { return sep; }
};

std::locale fileNameLocale(std::cout.getloc(), new punct_facet<char, ','>);

struct Ubx_Rxm_Rawx_Queued
{
	vector<unsigned char> message;
};

queue<Ubx_Rxm_Rawx_Queued> ubx_Rxm_Rawx_Queue;

struct MyPointCloud
{
  sensor_msgs::PointCloud2ConstPtr pointCloudPtr;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
};

struct InfoPointCloudToSave
{
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  string fileInfo;
};

queue<InfoPointCloudToSave> pointCloudsToSave;

struct HMIData
{
    int sat;
    int rtk;
    int st;
    int utc;
    int ins;
    double lon;
    double lat;
    double alt;
    int pcd;
    int pc;
    float y_u;
    float p_u;
    float r_u;
    float posu;
};

HMIData hmiData;

long mapping_count;

std::map<uint32_t,MyPointCloud> lidar_datas;
//std::map<uint32_t,nav_msgs::Odometry> imu_datas;
//std::map<uint32_t,sensor_msgs::NavSatFix> rtk_datas;
std::map<uint32_t,livox_mapping::GSOF> gsof_datas;
//uint32_t *count_lidar = new uint32_t(0);
//uint32_t *count_imu = new uint32_t(0); 
//uint32_t *count_rtk = new uint32_t(0);
uint32_t count_lidar = 0;
//uint32_t count_imu = 0; 
//uint32_t count_rtk = 0;
uint32_t count_gsof = 0;
uint32_t lidar_remove_from = 0;
//uint32_t imu_remove_from = 0;
//uint32_t rtk_remove_from = 0;
uint32_t gsof_remove_from = 0;

double lidar_delta_time = 0.1;//100Hz lidar data, you can change this parameter by your sensor

const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
Eigen::Matrix4d rtk2lidar; //the extrinsic parameter
ros::Publisher pub_cloud, pub_odometry;
ros::Publisher pub_apx;
ros::Publisher pub_apx_p2;
ros::Publisher pub_apx_rpy;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());
int pcdFilesCounter = 0;
int execution = 0;
int hmiBtnState = 0;

char *portname;
int comPort;

rosbag::Bag bag;

int setupComAttrs ( int fd, int speed )
{
	struct termios options;
	memset ( &options, 0, sizeof options );

	if ( tcgetattr ( fd, &options ) != 0 )
	{
		cout << "HMISupport::SetInterfaceAttr:error " << errno << " from tcgetattr" << endl;
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
		cout << "HMISupport::SetInterfaceAttr:error " << errno << " from tcsetattr" << endl;
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
		cout << "error " << errno << " from tggetattr" << endl;
		return;
	}

	options.c_cc[VMIN]  = should_block ? 1 : 0;
	options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if ( tcsetattr ( fd, TCSANOW, &options ) != 0 )
		cout << "error " << errno << " setting term attributes" << endl;
}

void setupComPort()
{
	cout << "HMISupport::SetupComPort:Opening COM Port " << portname << " at 115200" << endl;

	comPort = open ( portname, O_RDWR | O_NOCTTY );
	if ( comPort < 0 )
	{
		cout << "HMISupport::SetupComPort:error " << errno << " opening " << portname << ": " << strerror ( errno ) << endl;
		throw std::exception();
		return;
	}

	setupComAttrs ( comPort, B115200 ); // set speed to 115200 bps, 8n1 (no parity)
	setComBlocking ( comPort, 0 );    // set no blocking
}

//bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_imu, uint32_t num_rtk);
bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_gsof);
//bool average_quaternion(nav_msgs::Odometry &start, nav_msgs::Odometry &end, Eigen::Quaterniond &result, double t);
bool average_quaternion(livox_mapping::GSOF &start, livox_mapping::GSOF &end, Eigen::Quaterniond &result, double t);
bool mercator_proj(double B_0, double L0, double B, double L, double &X, double&Y);
void RGBTrans(PointType const * const pi, pcl::PointXYZRGB * const po);

inline double to_time(sensor_msgs::PointCloud2ConstPtr lidar_data)
{   
    try{
        return (lidar_data->header.stamp.sec )*1.0 + (lidar_data->header.stamp.nsec / 1000000000.0);
    }catch(exception& ex){
        cout << "\n **********to_time(sensor_msgs::PointCloud2ConstPtr lidar_data)************************************ \n" 
        << ex.what() 
        << "\n ***************to_time(sensor_msgs::PointCloud2ConstPtr lidar_data)******************************* \n" 
        << endl;
        return 0.0;
    }
}
/*inline double to_time(nav_msgs::Odometry imu_data)
{
    return (imu_data.header.stamp.sec )*1.0 + (imu_data.header.stamp.nsec / 1000000000.0);
}*/
/*inline double to_time(sensor_msgs::NavSatFix rtk_data)
{
    return (rtk_data.header.stamp.sec )*1.0 + (rtk_data.header.stamp.nsec / 1000000000.0);
}*/
inline double to_time(livox_mapping::GSOF gsof_data)
{
    try{
        return (gsof_data.sec )*1.0 + (gsof_data.nsec / 1000000000.0);
    }catch(exception& ex){
        cout << "\n ****************to_time(livox_mapping::GSOF gsof_data)****************************** \n" 
        << ex.what() 
        << "\n *****************to_time(livox_mapping::GSOF gsof_data)***************************** \n" 
        << endl;
        return 0.0;
    }
}
/*-----------------------------------------------------customMsg---------------------------*/
inline double to_time(livox_mapping::CustomMsg lidar_data)
{
    try{
        // return (lidar_data.timebase / 1000000 % 10000000) / 1000.0;
        return lidar_data.timebase / 1000000000.0;
    }catch(exception& ex){
        cout << "\n ********************to_time(livox_mapping::CustomMsg lidar_data)************************** \n" 
        << ex.what() 
        << "\n ********************to_time(livox_mapping::CustomMsg lidar_data)************************** \n" 
        << endl;
        return 0.0;
    }
}
void apxCbk(const livox_mapping::CustomMsg::ConstPtr &msg)
{
    try{
        pcl::PointCloud<PointType> laserCloud;

        pcl::PointCloud<PointType>::Ptr laserCloud2(new pcl::PointCloud<PointType>());
        
        laserCloud2->is_dense = false;
        laserCloud2->height = 1;
        laserCloud2->width = msg->point_num;
        laserCloud2->points.resize(msg->point_num);

        //std::cout<<"DEBUG apxCbk timebase "<< msg->timebase <<std::endl;

        for(int i = 0; i < msg->point_num; i++){
            laserCloud2->points[i].x = msg->points[i].x;
            laserCloud2->points[i].y = msg->points[i].y;
            laserCloud2->points[i].z = msg->points[i].z;
            laserCloud2->points[i].intensity = msg->points[i].reflectivity;
        }

        sensor_msgs::PointCloud2 p2_apx;

        pcl::toROSMsg(*laserCloud2, p2_apx);
        p2_apx.header.frame_id = "camera_init";
        p2_apx.header.stamp = msg->header.stamp;
        pub_apx_p2.publish(p2_apx);
    }catch(exception& ex){
        cout << "\n *********************apxCbk************************* \n" 
        << ex.what() 
        << "\n ************************apxCbk********************** \n" 
        << endl;
    }
}
//-------------------------------------------------------------------------------------------

void lidarCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    try{
        if(count_lidar > 0)
        {
            if(to_time(lidar_datas[count_lidar-1].pointCloudPtr) > to_time(msg))
            {
                ROS_INFO("lidar time error");
                return;
            }
        }
        if(execution == 1)
        {
            if(hmiBtnState == 1){
                bag.write("/livox/lidar", ros::Time::now(), *msg);
            }

            time_t pcTime = time(NULL);
            tm *currentTime = localtime(&pcTime);

            MyPointCloud myPointCloud;
            myPointCloud.pointCloudPtr = msg;
            myPointCloud.year = 1900+currentTime->tm_year;
            myPointCloud.month = 1+currentTime->tm_mon;
            myPointCloud.day = currentTime->tm_mday;
            myPointCloud.hour = currentTime->tm_hour;
            myPointCloud.minute = currentTime->tm_min;
            myPointCloud.second = currentTime->tm_sec;
            lidar_datas.emplace(count_lidar++, myPointCloud);
        }
    }catch(exception& ex){
        cout << "\n *********************lidarCbk************************* \n" 
        << ex.what() 
        << "\n ************************lidarCbk********************** \n" 
        << endl;
    }
}

/*void imuCbk(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(execution == 1)
        imu_datas.emplace(count_imu++, *msg);
}*/
/*void apximuCbk(const sensor_msgs::Imu::ConstPtr &msg) // transform sensor_msgs::Imu to nav_msgs::Odometry
{
    nav_msgs::Odometry tempOdo;
    tempOdo.header.stamp = msg->header.stamp;
    tempOdo.pose.pose.orientation.x = msg->orientation.x;
    tempOdo.pose.pose.orientation.y = msg->orientation.y;
    tempOdo.pose.pose.orientation.z = msg->orientation.z;
    tempOdo.pose.pose.orientation.w = msg->orientation.w;
    tempOdo.header.frame_id = "/camera_init";

    pub_apx_rpy.publish(tempOdo);
}*/

double currentTime;
int currentIns;
void gsofCbk(const livox_mapping::GSOF::ConstPtr &msg){
    try{

        hmiData.lon = msg->longitude;
        hmiData.lat = msg->latitude;
        hmiData.alt = msg->altitude;
        hmiData.ins = msg->insstatus & 0b11; //INS Status
        hmiData.r_u = msg->rollu;
        hmiData.p_u = msg->pitchu;
        hmiData.y_u = msg->yawu;
        hmiData.posu = msg->posu;

        currentTime = to_time(*msg);
        currentIns = hmiData.ins; //INS Status

        nav_msgs::Odometry tempOdo;
        tempOdo.header.stamp.sec = msg->sec;
        tempOdo.header.stamp.nsec = msg->nsec;
        tempOdo.pose.pose.orientation.x = msg->orientation.x;
        tempOdo.pose.pose.orientation.y = msg->orientation.y;
        tempOdo.pose.pose.orientation.z = msg->orientation.z;
        tempOdo.pose.pose.orientation.w = msg->orientation.w;
        tempOdo.header.frame_id = "/camera_init";

        pub_apx_rpy.publish(tempOdo);

        if(execution == 1){
            if(hmiBtnState == 1){
                bag.write("/gnss_inertial/gsof", ros::Time::now(), *msg);
            }
            gsof_datas.emplace(count_gsof++, *msg);
        }
            

    }catch(exception& ex){
        cout << "\n ********************gsofCbk************************** \n" 
        << ex.what() 
        << "\n ************************gsofCbk********************** \n" 
        << endl;
    }
}
/*void rtkCbk(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    currentTime = to_time(*msg);
    currentIns = (int)msg->status.service & 0b11; //INS Status

    hmiData.lon = msg->longitude;
    hmiData.lat = msg->latitude;
    hmiData.alt = msg->altitude;
    hmiData.st = (int)msg->status.status; //GPS Status
    hmiData.ins = (int)msg->status.service & 0b11; //INS Status
    if(execution == 1)
        rtk_datas.emplace(count_rtk++, *msg);
}*/

void gnssInfoCbk(const livox_mapping::GNSS::ConstPtr &msg)
{
    try{
        hmiData.sat = msg->sat;
        hmiData.st = msg->fix;
        hmiData.rtk = msg->rtk;
    }catch(exception& ex){
        cout << "\n *******************gnssInfoCbk*************************** \n" 
        << ex.what() 
        << "\n **********************gnssInfoCbk************************ \n" 
        << endl;
    }
}

void offsetFlagCbk(const std_msgs::Int32::ConstPtr &msg)
{
    try{
        hmiData.utc = (int)msg->data;
    }catch(exception& ex){
        cout << "\n ******************offsetFlagCbk**************************** \n" 
        << ex.what() 
        << "\n ***********************offsetFlagCbk*********************** \n" 
        << endl;
    }
}

//int recibidos = 0;
void rawUbxCbk(const livox_mapping::RawUbx::ConstPtr &msg)
{
    cout << "ubxRawData.size(): " << msg->ubxRawData.size() << endl;
    if(execution == 1){
        //cout << "recibidos: " << ++recibidos << "  " << msg->ubxRawData.size() << endl;
        Ubx_Rxm_Rawx_Queued ubx_Rxm_Rawx_Queued;
        ubx_Rxm_Rawx_Queued.message = msg->ubxRawData;
        ubx_Rxm_Rawx_Queue.push(ubx_Rxm_Rawx_Queued);
    }
}

std::string map_file_path;
pthread_mutex_t pcdQueue;

void *savePcdFilesThread(void *vargp)
{
	InfoPointCloudToSave pointCloudToSave;
    pcl::PCDWriter pcd_writer;
    pthread_mutex_lock(&pcdQueue);
    int sz = pointCloudsToSave.size();
    pthread_mutex_unlock(&pcdQueue);
	while(ros::ok() || sz > 0)
	{
		if(sz > 0)
		{
            try{
                pointCloudToSave = pointCloudsToSave.front();
                //pcl::PCDWriter pcd_writer;
                //pcd_writer.writeASCII(pointCloudToSave.fileInfo, *pointCloudToSave.pointCloud);
                //pcd_writer.writeBinary(pointCloudToSave.fileInfo, *pointCloudToSave.pointCloud);
                pcd_writer.writeBinary(pointCloudToSave.fileInfo, pointCloudToSave.pointCloud);
                //pcd_writer.writeASCII(pointCloudToSave.fileInfo, pointCloudToSave.pointCloud);
                std::cout << "Guardó archivo pcd " << pointCloudToSave.fileInfo << std::endl;
                pointCloudsToSave.pop();
            }catch(exception& ex){
                cout << "\n ********************savePcdFilesThread************************** \n" 
                << ex.what() 
                << "\n ***********************savePcdFilesThread*********************** \n" 
                << endl;
            }
		}
		pthread_mutex_lock(&pcdQueue);
        sz = pointCloudsToSave.size();
        pthread_mutex_unlock(&pcdQueue);
        hmiData.pc = sz;
        //cout << "size hilo: " << sz << endl;
	}
	return NULL;
}

void *readComPort(void *vargp)
{
    char commands[100];
    int readBytes;
    while(ros::ok())
    {
        try{
            //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            //continue;

            readBytes = read(comPort, commands, sizeof(commands));
            if(readBytes != 0 && readBytes!=-1)
            {
                if(strstr(commands, "PLAY"))
                {
                    execution = 1;
                }
                else if(strstr(commands, "STOP"))
                {
                    execution = 0;
                }
            }
        }catch(exception& ex){
            cout << "\n ******************readComPort**************************** \n" 
            << ex.what() 
            << "\n *********************readComPort************************* \n" 
            << endl;
        }
    }
    return NULL;
}

void *updateHMI(void *vargp)
{
	while(ros::ok())
	{
        try{
            //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            //continue;

            string command;
            //NumSats
            stringstream satConcat;
            satConcat << "sat.txt=\"" << hmiData.sat << "\"" << char(255) << char(255) << char(255);
            //command = satConcat.str();
            //write(comPort, command.c_str(), command.length());

            //utc offset flag
            //stringstream utcConcat;
            //utcConcat << "utc.txt=\"" << hmiData.utc << "\"" << char(255) << char(255) << char(255);
            satConcat << "utc.txt=\"" << hmiData.utc /*hmiBtnState*/ << "\"" << char(255) << char(255) << char(255);
            //command = utcConcat.str();
            //write(comPort, command.c_str(), command.length());

            //ins status
            //stringstream insConcat;
            //insConcat << "ins.txt=\"" << hmiData.ins << "\"" << char(255) << char(255) << char(255);
            satConcat << "ins.txt=\"" << hmiData.ins << "\"" << char(255) << char(255) << char(255);
            //command = insConcat.str();
            //write(comPort, command.c_str(), command.length());

            //gps status
            /*stringstream stConcat;
            stConcat << "st.txt=\"" << hmiData.st << "\"" << char(255) << char(255) << char(255);
            command = stConcat.str();
            write(comPort, command.c_str(), command.length());*/
            satConcat << "st.txt=\"" << hmiData.st << "\"" << char(255) << char(255) << char(255);

            //pcd files
            /*stringstream pcdConcat;
            pcdConcat << "pcd.txt=\"" << hmiData.pcd << "\"" << char(255) << char(255) << char(255);
            command = pcdConcat.str();
            write(comPort, command.c_str(), command.length());*/
            satConcat << "pcd.txt=\"" << hmiData.pc << "/" << hmiData.pcd << "\"" << char(255) << char(255) << char(255);

            //longitude
            /*stringstream lonConcat;
            lonConcat << std::setprecision(12) << "lon.txt=\"" << hmiData.lon << "\"" << char(255) << char(255) << char(255);
            command = lonConcat.str();
            write(comPort, command.c_str(), command.length());*/
            satConcat << std::setprecision(12) << "lon.txt=\"" << hmiData.lon << "\"" << char(255) << char(255) << char(255);

            //gps status
            /*stringstream latConcat;
            latConcat << std::setprecision(12) << "lat.txt=\"" << hmiData.lat << "\"" << char(255) << char(255) << char(255);
            command = latConcat.str();
            write(comPort, command.c_str(), command.length());*/
            satConcat << std::setprecision(12) << "lat.txt=\"" << hmiData.lat << "\"" << char(255) << char(255) << char(255);

            //gps status
            /*stringstream altConcat;
            altConcat << std::setprecision(12) << "alt.txt=\"" << hmiData.alt << "\"" << char(255) << char(255) << char(255);
            command = altConcat.str();
            write(comPort, command.c_str(), command.length());*/
            satConcat << std::setprecision(12) << "alt.txt=\"" << hmiData.alt << "\"" << char(255) << char(255) << char(255);

            /*stringstream btnConcat;
            //pthread_mutex_lock(&hmiBtnStateMutex);
            btnConcat << "b0.picc=" << hmiBtnState << char(255) << char(255) << char(255);
            //pthread_mutex_unlock(&hmiBtnStateMutex);*/
            satConcat << "b0.picc=" << hmiBtnState << char(255) << char(255) << char(255);
            //command = btnConcat.str();

            //rtk
            satConcat << "rtk.txt=\"" << hmiData.rtk << "\"" << char(255) << char(255) << char(255);
            //posu
            satConcat << std::setprecision(12) << "posu.txt=\"" << hmiData.posu << "\"" << char(255) << char(255) << char(255);
            //yawu
            satConcat << std::setprecision(12) << "yu.txt=\"" << hmiData.y_u << "\"" << char(255) << char(255) << char(255);
            //pitchu
            satConcat << std::setprecision(12) << "pu.txt=\"" << hmiData.p_u << "\"" << char(255) << char(255) << char(255);
            //rollu
            satConcat << std::setprecision(12) << "ru.txt=\"" << hmiData.r_u << "\"" << char(255) << char(255) << char(255);

            command = satConcat.str();
            write(comPort, command.c_str(), command.length());
            
            cout << "HMIBtnState: " << hmiBtnState << endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }catch(exception& ex){
            cout << "\n ********************updateHMI************************** \n" 
            << ex.what() 
            << "\n *********************updateHMI************************* \n" 
            << endl;
        }
	}
	return NULL;
}

pthread_mutex_t rawUbxMutex;
char * logUbxPath;
FILE * pFile;//ubx file
//int ubxLogFile;
void *writeUbxRxmRawxFile(void *vargp)
{
    pthread_mutex_lock(&rawUbxMutex);
    int sz1 = ubx_Rxm_Rawx_Queue.size();
    pthread_mutex_unlock(&rawUbxMutex);
	Ubx_Rxm_Rawx_Queued ubx_Rxm_Rawx_Queued;
    //int guardados = 0;
	while(ros::ok() || sz1 > 0)
	{
		if(sz1 > 0 && hmiBtnState == 1)
		{
		    ubx_Rxm_Rawx_Queued = ubx_Rxm_Rawx_Queue.front();

		    //FILE * pFile;
			//pFile = fopen (logUbxPath, "ab");
            int len = ubx_Rxm_Rawx_Queued.message.size();
            unsigned char message[len];
            for(int i = 0; i < len; i++){
                message[i] = ubx_Rxm_Rawx_Queued.message[i];
            }
		    fwrite (message, sizeof(unsigned char), len, pFile);
			//fclose (pFile);
            //cout << "guardados: " << ++guardados << endl;
			//int ubxLogFile = open ( "/home/srcc/logfile.ubx", O_WRONLY | O_CREAT, S_IRWXU );
			//write(ubxLogFile, ubx_Rxm_Rawx_Queue.front().message, ubx_Rxm_Rawx_Queue.front().len);
			//free(ubx_Rxm_Rawx_Queued.message);
			//close(ubxLogFile);
		    ubx_Rxm_Rawx_Queue.pop();
		}
        pthread_mutex_lock(&rawUbxMutex);
        sz1 = ubx_Rxm_Rawx_Queue.size();
        pthread_mutex_unlock(&rawUbxMutex);
	}
	return NULL;
}

bool displayPortFound = false;
//findPorts: /home/srcc/Documents/Livox/ws_livox/checkDev.sh
void findPorts()
{
	string result = exec ( "/home/srcc/Documents/Livox/ws_livox/checkDev.sh" );
	
	char buffer[result.length()];
	strcpy(buffer, result.c_str());
		
	char *deviceArray[30];
	char *deviceParts[5];
	
	int deviceCnt = mkarray ( deviceArray, buffer, "\n" );
	for (int i=0;i<deviceCnt;i++)
	{
		int partCnt = mkarray ( deviceParts, deviceArray[i], " " );
		
		if (strcmp(deviceParts[2],"FTDI_FT232R_USB_UART_A50285BI") == 0)
		{
            int len = strlen(deviceParts[0])+1;
            portname = (char *)malloc(len);
			strcpy(portname,deviceParts[0]);
			printf("findPorts- Found DisplayPort on %s\n",portname);
            displayPortFound = true;
		}
	}
}

int main(int argc, char **argv)
{
    //update the extrinsic parameter
    rtk2lidar <<  -1,0,0,0,
                  0,-1,0,0,
                  0,0, 1,0,
                  0,0,0, 1;

    //pFile = fopen ("/home/srcc/logfile.ubx", "wb");//ubx file
    //ubxLogFile = open ( "/home/srcc/logfile.ubx", O_WRONLY | O_CREAT, S_IRWXU );

    ros::init(argc, argv, "livox_mapping_case");
    ros::NodeHandle n;

    //ros::AsyncSpinner spinner(0);
    //spinner.start();

    /*string hmiport;
    ros::param::get("~hmiport", hmiport);
    portname = (char *)malloc(hmiport.length()+1);
    strcpy(portname, hmiport.c_str());
    printf("HMI Port: %s\n", portname);*/
    findPorts();//HMI Port

    if(displayPortFound)
        setupComPort(); //HMI Port

    //displayPortFound = true;

    ros::Subscriber sub_apx_gsof = n.subscribe("/gnss_inertial/gsof", 20000, gsofCbk);
    //ros::Subscriber sub_rtk = n.subscribe("/gnss_inertial/navsatfix", 1000, rtkCbk);
    //ros::Subscriber sub_apx_rpy = n.subscribe("/gnss_inertial/imu", 20000, apximuCbk);

    ros::Subscriber sub_gnss = n.subscribe("/gnss_inertial/gnss_info", 1000, gnssInfoCbk);
    ros::Subscriber sub_offsetflag = n.subscribe("/gnss_inertial/offsetflag", 1000, offsetFlagCbk);
    ros::Subscriber sub_rawubx = n.subscribe("/gnss_inertial/rawubx", 20000, rawUbxCbk);

    pub_apx_rpy = n.advertise<nav_msgs::Odometry> ("pub_apx_rpy", 1000);

    // ros::Subscriber sub_apx = n.subscribe("/livox/lidar", 1000, apxCbk);
    // pub_apx_p2 = n.advertise<sensor_msgs::PointCloud2>("pub_point2_apx", 1000);
    // ros::Subscriber sub_point = n.subscribe("pub_point2_apx", 1000, lidarCbk);

    ros::Subscriber sub_point = n.subscribe("/livox/lidar", 1000, lidarCbk);
    //ros::Subscriber sub_imu = n.subscribe("pub_apx_rpy", 20000, imuCbk);

    pub_cloud = n.advertise<sensor_msgs::PointCloud2>("pub_pointcloud2", 1);
    pub_odometry = n.advertise<nav_msgs::Odometry>("pub_odometry", 1);

    std::string map_file_path_temp;
    ros::param::get("~map_file_path",map_file_path_temp);

    double publish_freq;
    ros::param::get("~publish_freq", publish_freq);
    cout << "publish_freq: " << publish_freq << endl;
    lidar_delta_time = 1.0 / publish_freq;
    cout << "lidar_delta_time: " << lidar_delta_time << endl;
    
    FILE *file;
    if(file = fopen("mapping_count.cnt", "r"))
    {
        int readElements = fread(&mapping_count, sizeof(mapping_count), 1, file);
        if(readElements == 0)
        {
            mapping_count = 0;
        }
        fclose(file);
    }else{
        mapping_count = 0;
    }

    pthread_t thread_id;
    pthread_t thread_hmi_id;
    pthread_t thread_hmi_read_id;
    pthread_t thread_rawubx_id;
    pthread_create(&thread_id, NULL, savePcdFilesThread, NULL);
    if(displayPortFound){
        pthread_create(&thread_hmi_id, NULL, updateHMI, NULL);
        pthread_create(&thread_hmi_read_id, NULL, readComPort, NULL);
    }
    pthread_create(&thread_rawubx_id, NULL, writeUbxRxmRawxFile, NULL);

    if(pthread_mutex_init(&pcdQueue, NULL) != 0)
    {
        cout << "Mutex init has failed!" << endl;
        return 1;
    }
    if(pthread_mutex_init(&rawUbxMutex, NULL) != 0)
    {
        cout << "RawUbx Mutex init has failed!" << endl;
        return 1;
    }

    while(n.ok())
    {
        //cout << "HMIButton: " << hmiBtnState << endl;
        ros::spinOnce();
        //execution = 1;
        if(execution == 1)
        {
            stringstream logFileConcat;

            logFileConcat << map_file_path_temp << "/Mapeo_" << mapping_count;

            // Creating a directory
            string mappingDir =  logFileConcat.str();
            if (mkdir(mappingDir.c_str(), 0777) == -1)
            {
                cerr << "Error creating dir: " << mappingDir << ". Error: " << strerror(errno) << endl;
                mapping_count++;
                execution = 0;
                continue;
            }
            else
            {
                cout << "Directory " << mappingDir << " created" << endl; 
            }

            logFileConcat << "/PCD_Files";
            map_file_path = logFileConcat.str();
            if (mkdir(map_file_path.c_str(), 0777) == -1)
            {
                cerr << "Error creating dir: " << map_file_path << ". Error: " << strerror(errno) << endl;
                execution = 0;
                continue;
            }
            else
            {
                cout << "Directory " << map_file_path << " created" << endl; 
            }

            stringstream logUbxConcat;
            logUbxConcat << mappingDir << "/LogRawUbx.ubx";
            string logUbxPathString = logUbxConcat.str();
            free(logUbxPath);
            logUbxPath = (char *)malloc(logUbxPathString.length()+1);
            strcpy(logUbxPath, logUbxPathString.c_str());
            printf("Log Ubx Path: %s\n", logUbxPath);
            pFile = fopen (logUbxPath, "wb");

            uint32_t num_lidar = 0;
            //uint32_t num_imu = 1;
            //uint32_t num_rtk = 1;
            uint32_t num_gsof = 1;
            
            uint32_t num_lidar_last = 0;
            //uint32_t num_imu_last = 1;
            //uint32_t num_rtk_last = 1;
            uint32_t num_gsof_last = 1;

            count_lidar = 0;
            //count_imu = 0; 
            //count_rtk = 0;
            count_gsof = 0;
            lidar_remove_from = 0;
            //imu_remove_from = 0;
            //rtk_remove_from = 0;
            gsof_remove_from = 0;

            pcdFilesCounter = 0;

            lidar_datas.clear();
            //imu_datas.clear();
            //rtk_datas.clear();
            gsof_datas.clear();

            bool init_flag = false; 

            stringstream rosbagFilePathConcat;
            rosbagFilePathConcat << mappingDir << "/rosbag.bag";
            
            bag.open(rosbagFilePathConcat.str(), rosbag::bagmode::Write);

            hmiBtnState = 1;

            //timestamp align
            while(n.ok() && execution == 1)
            {
                try{
                    //cout << "HMIButton: " << hmiBtnState << endl;
                    ros::spinOnce();
                    if(num_lidar < count_lidar)
                    {
                        //bool imu_flag = false;
                        //bool rtk_flag = false;
                        bool gsof_flag = false;
                        
                        // imu data align
                        /*if(num_imu < count_imu)
                        {
                            if(to_time(imu_datas[num_imu-1]) <= to_time(lidar_datas[num_lidar].pointCloudPtr))
                            {
                                if(to_time(imu_datas[num_imu]) >= to_time(lidar_datas[num_lidar].pointCloudPtr))
                                {
                                    imu_flag = true;   
                                }
                                else
                                {
                                    num_imu++;
                                }
                            }
                            else
                            {
                                num_lidar++;
                                continue;
                            }
                        }*/

                        // rtk data align
                        /*if(num_rtk < count_rtk)
                        {
                            if(to_time(rtk_datas[num_rtk-1]) <= to_time(lidar_datas[num_lidar].pointCloudPtr))
                            {
                                if(to_time(rtk_datas[num_rtk]) >= to_time(lidar_datas[num_lidar].pointCloudPtr))
                                {
                                    rtk_flag = true;
                                }
                                else
                                {
                                    num_rtk++;
                                }
                            }
                            else
                            {
                                num_lidar++;
                                continue;
                            }
                        }*/

                        if(num_gsof < count_gsof)
                        {
                            if(to_time(gsof_datas[num_gsof-1]) <= to_time(lidar_datas[num_lidar].pointCloudPtr))
                            {
                                if(to_time(gsof_datas[num_gsof]) >= to_time(lidar_datas[num_lidar].pointCloudPtr))
                                {
                                    gsof_flag = true;   
                                }
                                else
                                {
                                    num_gsof++;
                                }
                            }
                            else
                            {
                                num_lidar++;
                                continue;
                            }
                        }

                    
                        if(gsof_flag)
                        {
                            if(init_flag)
                            {
                                /*std::cout << "totales: " << count_lidar << " " << count_gsof << std::endl;
                                std::cout << "remove: " << lidar_remove_from << " " << gsof_remove_from << std::endl;
                                std::cout << "last: " << num_lidar_last << " " << num_gsof_last << std::endl;
                                std::cout << "contadores: " << num_lidar << " " << num_gsof << std::endl;
                                std::cout << "vectores: " << lidar_datas.size() << " " << gsof_datas.size() << std::endl;
                                */

                                //if(!lidar_imu_rtk_process(num_lidar_last, num_imu_last-1, num_rtk_last))
                                if(!lidar_imu_rtk_process(num_lidar_last, num_gsof_last))
                                {
                                    cout << "error happened" << endl;
                                    return -1;
                                }
                                
                                num_lidar_last = num_lidar;
                                //num_imu_last = num_imu;
                                //num_rtk_last = num_rtk;
                                num_gsof_last = num_gsof;
                                for(uint32_t i = lidar_remove_from; i < num_lidar_last-1; i++)
                                {
                                    lidar_datas.erase(i);
                                }
                                
                                /*for(uint32_t i = imu_remove_from; i < num_imu_last-1; i++)
                                {
                                    imu_datas.erase(i);
                                }
                                for(uint32_t i = rtk_remove_from; i < num_rtk_last-1; i++)
                                {
                                    rtk_datas.erase(i);
                                }*/

                                for(uint32_t i = gsof_remove_from; i < num_gsof_last-1; i++)
                                {
                                    gsof_datas.erase(i);
                                }
                                lidar_remove_from = num_lidar_last-1;
                                //imu_remove_from = num_imu_last-1;
                                //rtk_remove_from = num_rtk_last-1;
                                gsof_remove_from = num_gsof_last-1;
                            }
                            else
                            {
                                init_flag = true;
                                num_lidar_last = num_lidar;
                                //num_imu_last = num_imu;
                                //num_rtk_last = num_rtk;
                                num_gsof_last = num_gsof;
                            }
                            num_lidar++;
                        }
                    }
                }catch(exception& ex){
                    cout << "\n ********************main************************** \n" 
                    << ex.what() 
                    << "\n ***********************main*********************** \n" 
                    << endl;
                }
            }
            pthread_mutex_lock(&pcdQueue);
            int sz = pointCloudsToSave.size();
            pthread_mutex_unlock(&pcdQueue);
            while(sz > 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                pthread_mutex_lock(&pcdQueue);
                sz = pointCloudsToSave.size();
                pthread_mutex_unlock(&pcdQueue);
                cout << "size main: " << sz << endl;
            } 
            pthread_mutex_lock(&rawUbxMutex);
            int sz1 = ubx_Rxm_Rawx_Queue.size();
            pthread_mutex_unlock(&rawUbxMutex);
            while(sz1 > 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                pthread_mutex_lock(&rawUbxMutex);
                sz1 = ubx_Rxm_Rawx_Queue.size();
                pthread_mutex_unlock(&rawUbxMutex);
                cout << "size main RawUbx: " << sz1 << endl;
            }
            
            hmiBtnState = 0;
            bag.close();
            fclose(pFile);
            
            mapping_count++;
            if(file = fopen("mapping_count.cnt", "w"))
            {
                int writtenElements = fwrite(&mapping_count, sizeof(mapping_count), 1, file);
                if(writtenElements == 0)
                {
                    cout << "Error al escribir el contador de mapeos al archivo." << endl;
                }
                fclose(file);
            }else{
                cout << "Error al abrir el archivo para guardar el contador de mapeos." << endl;
            }
            cout << "Mapping Count: " << mapping_count << endl;
        }
    }
	
    pthread_join(thread_id, NULL);
    
    if(displayPortFound){
        pthread_join(thread_hmi_id, NULL);
        pthread_join(thread_hmi_read_id, NULL);
    }

    pthread_join(thread_rawubx_id, NULL);

    close(comPort);

    //ros::waitForShutdown();

    //fclose (pFile); //ubx file
	//close(ubxLogFile);

    //cout << "pointCloudsToSave.size(): " << pointCloudsToSave.size() << endl;

    /*std::string all_points_filename(map_file_path + "/all_points.pcd");
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeASCII(all_points_filename, *laserCloudFullResColor_pcd);
    std::cout << "Guardó archivo pcd" << std::endl;*/

    return 0;
}

//bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_imu, uint32_t num_rtk)
bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_gsof)
{
    try{
        // std::cout<<setprecision(20);
        // std::cout<<"DEBUG lidar_datas : "<< to_time(lidar_datas[num_lidar]) << std::endl;
        // std::cout<<"DEBUG to_time(imu_datas[num_imu]) : "<< to_time(imu_datas[num_imu]) << std::endl;
        // std::cout<<"DEBUG to_time(imu_datas[num_imu+1]) : "<< to_time(imu_datas[num_imu+1]) << std::endl;

        pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
        pcl::fromROSMsg(*lidar_datas[num_lidar].pointCloudPtr, laserCloudIn);

        double lidar_t = to_time(lidar_datas[num_lidar].pointCloudPtr);
        //double imu_back_t = to_time(imu_datas[num_imu+1]);
        //double rtk_back_t = to_time(rtk_datas[num_rtk]);
        double gsof_back_t = to_time(gsof_datas[num_gsof]);

        //std::cout << "totales: " << count_lidar << " " << count_imu << " " << count_rtk << std::endl;
        //std::cout << "remove: " << lidar_remove_from << " " << imu_remove_from << " " << rtk_remove_from << std::endl;
        //std::cout << "contadores: " << num_lidar << " " << num_imu << " " << num_rtk << std::endl;
        //std::cout << "vectores: " << lidar_datas.size() << " " << imu_datas.size() << " " << rtk_datas.size() << std::endl;

        //std::cout<<"DEBUG laserCloudIn " << laserCloudIn.points.size() << std::endl;
        double dt = lidar_delta_time / laserCloudIn.points.size();


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());

        Eigen::Matrix3d rot;
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

        //with Distortion correction
        for(int i=0; i<laserCloudIn.points.size(); i++)
        {
            // if over time use the next
            /*if(lidar_t > imu_back_t)
            {
                num_imu++;
                imu_back_t = to_time(imu_datas[num_imu + 1]);
            }
            
            if(lidar_t > rtk_back_t)
            {
                num_rtk++;
                rtk_back_t = to_time(rtk_datas[num_rtk]);
            }*/

            if(lidar_t > gsof_back_t)
            {
                num_gsof++;
                gsof_back_t = to_time(gsof_datas[num_gsof]);
            }

            /*int ins1 = (int)rtk_datas[num_rtk-1].status.service & 0b11;
            int gps1 = (int)rtk_datas[num_rtk-1].status.status;
            int ins2 = (int)rtk_datas[num_rtk].status.service & 0b11;
            int gps2 = (int)rtk_datas[num_rtk].status.status;

            double time1 = to_time(rtk_datas[num_rtk-1]);
            double time2 = to_time(rtk_datas[num_rtk]);*/

            //int ins1 = (int)gsof_datas[num_gsof-1].service & 0b11;
            //int gps1 = (int)gsof_datas[num_gsof-1].status;
            //int ins2 = (int)gsof_datas[num_gsof].service & 0b11;
            //int gps2 = (int)gsof_datas[num_gsof].status;

            //double time1 = to_time(gsof_datas[num_gsof-1]);
            //double time2 = to_time(gsof_datas[num_gsof]);

            /*cout << setprecision(20) << "Current Time: " << currentTime << endl;
            //cout << setprecision(20) << "Current Ins: " << currentIns << endl;
            cout << setprecision(20) << "GSOF  Time1: " << time1 << endl;
            cout << setprecision(20) << "GSOF  Time2: " << time2 << endl;
            cout << "lidar_imu_rtk_process cpu: " << sched_getcpu() << endl;*/

            int ins1 = (int)gsof_datas[num_gsof-1].insstatus & 0b11;
            int ins2 = (int)gsof_datas[num_gsof].insstatus & 0b11;

            /*NO INS FILTER OR GPS FIXED*/
            //if(ins1 != 2 || gps1 != 3 || ins2 != 2 || gps2 != 3)
            if(ins1 != 2 || ins2 != 2)
            {
                //cout << "BAD Coordinates: INS1: " << ins1 << " GPS1: " << gps1 << " INS2: " << ins2 << " GPS2: " << gps2 << endl;
                lidar_t += dt;
                continue;
            }
            else
            {
                //cout << "GOOD Coordinates: INS1: " << ins1 << " GPS1: " << gps1 << " INS2: " << ins2 << " GPS2: " << gps2 << endl;
            }
            /*NO INS FILTER OR GPS FIXED*/

            //double temp_t = lidar_t - to_time(imu_datas[num_imu]);
            //double t22 = to_time(imu_datas[num_imu+1]) - to_time(imu_datas[num_imu]);

            double temp_t = lidar_t - to_time(gsof_datas[num_gsof-1]);
            double t22 = to_time(gsof_datas[num_gsof]) - to_time(gsof_datas[num_gsof-1]);

            Eigen::Quaterniond q;

            //if(!average_quaternion(imu_datas[num_imu], imu_datas[num_imu+1], q, temp_t/t22)) //
            if(!average_quaternion(gsof_datas[num_gsof-1], gsof_datas[num_gsof], q, temp_t/t22)) //
            {
                continue;
            }
            rot = q.normalized().toRotationMatrix();

            double LLA[3],p[3];
            /*double t1 = lidar_t - to_time(rtk_datas[num_rtk-1]);
            double t2 = to_time(rtk_datas[num_rtk]) - to_time(rtk_datas[num_rtk-1]);
            LLA[0] = rtk_datas[num_rtk-1].longitude*(1-t1/t2) + rtk_datas[num_rtk].longitude*(t1/t2);
            LLA[1] = rtk_datas[num_rtk-1].latitude*(1-t1/t2) + rtk_datas[num_rtk].latitude*(t1/t2);
            LLA[2] = rtk_datas[num_rtk-1].altitude*(1-t1/t2) + rtk_datas[num_rtk].altitude*(t1/t2);*/

            double t1 = lidar_t - to_time(gsof_datas[num_gsof-1]);
            double t2 = to_time(gsof_datas[num_gsof]) - to_time(gsof_datas[num_gsof-1]);
            LLA[0] = gsof_datas[num_gsof-1].longitude*(1-t1/t2) + gsof_datas[num_gsof].longitude*(t1/t2);
            LLA[1] = gsof_datas[num_gsof-1].latitude*(1-t1/t2) + gsof_datas[num_gsof].latitude*(t1/t2);
            LLA[2] = gsof_datas[num_gsof-1].altitude*(1-t1/t2) + gsof_datas[num_gsof].altitude*(t1/t2);

            static const double LLA0[3] = {LLA[0], LLA[1], LLA[2]};
            //static const double LLA0[3] = {-74.0, 11.0, 10.0}; //prueba

            if(!mercator_proj(LLA0[1]*M_PI/180, LLA0[0]*M_PI/180, LLA[1]*M_PI/180, LLA[0]*M_PI/180, p[0], p[1]))
            {
                // Mercator projection
                continue;
            }
            p[2] = LLA[2];

            static const double p0[3] = {p[0], p[1], p[2]};
            
            p[0] = p[0] - p0[0];
            p[1] = p[1] - p0[1];
            p[2] = p0[2] - p[2];

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            
            Eigen::Vector3d cn;
            cn << p[0], p[1], p[2];

            T.block<3, 3>(0, 0) = rot;
            T.block<3, 1>(0, 3) =  cn;

            static const Eigen::Matrix4d T1 = T;

            trans = rtk2lidar * T1.inverse() * T * rtk2lidar.inverse();

            Eigen::Matrix<double,4,1> or_point,after_point;
            or_point << laserCloudIn.points[i].x, laserCloudIn.points[i].y, laserCloudIn.points[i].z, 1;

            //Eigen::Matrix4d trans_inv = trans.inverse();

            after_point = trans * or_point;

            pcl::PointXYZI temp_after_point;
            temp_after_point.x = after_point[0];
            temp_after_point.y = after_point[1];
            temp_after_point.z = after_point[2];
            temp_after_point.intensity = laserCloudIn.points[i].intensity;

            //temp_after_point.z = -temp_after_point.z; 

            pcl::PointXYZRGB temp_point;
            RGBTrans(&temp_after_point, &temp_point);
            laserCloudFullResColor->push_back(temp_point);

            lidar_t += dt;
        }

        // //no Distortion correction

        // double temp_t = lidar_t - to_time(imu_datas[num_imu]);
        // double t22 = to_time(imu_datas[num_imu+1]) - to_time(imu_datas[num_imu]);

        // Eigen::Quaterniond q;

        // if(!average_quaternion(imu_datas[num_imu], imu_datas[num_imu+1], q, temp_t/t22))
        // {
        //     return 0;
        // }
        // rot = q.normalized().toRotationMatrix();

        // double LLA[3],p[3];
        // double t1 = lidar_t - to_time(rtk_datas[num_rtk-1]);
        // double t2 = to_time(rtk_datas[num_rtk]) - to_time(rtk_datas[num_rtk-1]);
        // LLA[0] = rtk_datas[num_rtk-1].longitude*(1-t1/t2) + rtk_datas[num_rtk].longitude*(t1/t2);
        // LLA[1] = rtk_datas[num_rtk-1].latitude*(1-t1/t2) + rtk_datas[num_rtk].latitude*(t1/t2);
        // LLA[2] = rtk_datas[num_rtk-1].altitude*(1-t1/t2) + rtk_datas[num_rtk].altitude*(t1/t2);

        // static const double LLA0[3] = {LLA[0], LLA[1], LLA[2]};

        // if(!mercator_proj(LLA0[1]*M_PI/180, LLA0[0]*M_PI/180, LLA[1]*M_PI/180, LLA[0]*M_PI/180, p[0], p[1]))
        // {
        //     // Mercator projection
        //     return 0;
        // }
        // p[2] = LLA[2];

        // static const double p0[3] = {p[0], p[1], p[2]};
        
        // p[0] = p[0] - p0[0];
        // p[1] = p[1] - p0[1];
        // p[2] = p0[2] - p[2];

        // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // Eigen::Vector3d cn;
        // cn << p[0], p[1], p[2];

        // T.block<3, 3>(0, 0) = rot;
        // T.block<3, 1>(0, 3) =  cn;

        // static const Eigen::Matrix4d T1 = T;

        // trans = rtk2lidar * T1.inverse() * T * rtk2lidar.inverse();

        // pcl::transformPointCloud(laserCloudIn, laserCloudIn, trans);

        // for(int i=0; i<laserCloudIn.points.size(); i++){
        //     pcl::PointXYZRGB temp_point;

        //     RGBTrans(&laserCloudIn.points[i],&temp_point);
        //     //temp_point.z = - temp_point.z;
        //     laserCloudFullResColor->push_back(temp_point);

        // }
        // //-------------------------------------------------------------------------------------

        Eigen::Matrix3d rotation_R = trans.block<3,3>(0,0);
        Eigen::Quaterniond QQ(rotation_R);

        nav_msgs::Odometry odo_output;
        odo_output.header.frame_id = "camera_init";
        odo_output.child_frame_id = "/livox";
        odo_output.pose.pose.orientation.w = QQ.w();
        odo_output.pose.pose.orientation.x = QQ.x();
        odo_output.pose.pose.orientation.y = QQ.y();
        odo_output.pose.pose.orientation.z = QQ.z();
        odo_output.pose.pose.position.x = trans(0, 3);
        odo_output.pose.pose.position.y = trans(1, 3);
        odo_output.pose.pose.position.z = trans(2, 3);
        pub_odometry.publish(odo_output);

        static tf::TransformBroadcaster tfBroadcaster;
        tf::StampedTransform aftMappedTrans;
        
        //aftMappedTrans.stamp_ = ros::Time().fromSec();
        aftMappedTrans.setRotation(tf::Quaternion(QQ.x(), QQ.y(), QQ.z(), QQ.w()));
        aftMappedTrans.setOrigin(tf::Vector3(trans(0, 3),trans(1, 3),trans(2, 3)));

        aftMappedTrans.frame_id_ = "/camera_init";
        aftMappedTrans.child_frame_id_ = "/aft_mapped";

        tfBroadcaster.sendTransform(aftMappedTrans);
        
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*laserCloudFullResColor, output);
        output.header.frame_id = "camera_init";
        pub_cloud.publish(output);

        //*laserCloudFullResColor_pcd += *laserCloudFullResColor;

        if(laserCloudFullResColor->size() > 0) //Save file only if it has points
        {
            std::stringstream filenameConcat;
            filenameConcat.imbue(fileNameLocale);
            filenameConcat << std::setprecision(12) << map_file_path << "/points" << pcdFilesCounter++ << "__" 
            << lidar_datas[num_lidar].year << "-" << lidar_datas[num_lidar].month << "-" << lidar_datas[num_lidar].day << "_" << lidar_datas[num_lidar].hour << "-" << lidar_datas[num_lidar].minute << "-" << lidar_datas[num_lidar].second << "__"
            //<< rtk_datas[num_rtk].longitude << "_" << rtk_datas[num_rtk].latitude << "_" << rtk_datas[num_rtk].altitude << ".pcd";
            << gsof_datas[num_gsof].longitude << "_" << gsof_datas[num_gsof].latitude << "_" << gsof_datas[num_gsof].altitude << ".pcd";
            std::string filename = filenameConcat.str();

            hmiData.pcd = pcdFilesCounter;

            InfoPointCloudToSave infoPointCloudToSave;
            infoPointCloudToSave.pointCloud = *laserCloudFullResColor;
            infoPointCloudToSave.fileInfo = filename;

            pthread_mutex_lock(&pcdQueue);
            pointCloudsToSave.push(infoPointCloudToSave);
            pthread_mutex_unlock(&pcdQueue);
        }

        /*std::stringstream filePathConcat;
        filePathConcat << map_file_path << "/points_" << pcdFilesCounter << ".pcd";

        std::string all_points_filename = filePathConcat.str();
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeASCII(all_points_filename, *laserCloudFullResColor);
        std::cout << "Guardó archivo pcd " << pcdFilesCounter++ << std::endl;*/
        
        return true;    
    }catch(exception& ex){
        cout << "\n *********************lidar_imu_rtk_process************************* \n" 
        << ex.what() 
        << "\n *************************lidar_imu_rtk_process********************* \n" 
        << endl;
        return false;
    }
}

void RGBTrans(PointType const * const pi, pcl::PointXYZRGB * po)
{
    try{
        po->x = pi->x;
        po->y = pi->y;
        po->z = pi->z;

        int reflection_map = pi->intensity;

        if (reflection_map < 30)
        {
            int green = (reflection_map * 255 / 30);
            po->r = 0;
            po->g = green & 0xff;
            po->b = 0xff;
        }
        else if (reflection_map < 90)
        {
            int blue = (((90 - reflection_map) * 255) / 60);
            po->r = 0x0;
            po->g = 0xff;
            po->b = blue & 0xff;
        }
        else if (reflection_map < 150)
        {
            int red = ((reflection_map-90) * 255 / 60);
            po->r = red & 0xff;
            po->g = 0xff;
            po->b = 0x0;
        }
        else
        {
            int green = (((255-reflection_map) * 255) / (255-150));
            po->r = 0xff;
            po->g = green & 0xff;
            po->b = 0;
        }
    }catch(exception& ex){
        cout << "\n ********************RGBTrans************************** \n" 
        << ex.what() 
        << "\n ***********************RGBTrans*********************** \n" 
        << endl;
    }
}
//Quaternion interpolation 
//bool average_quaternion(nav_msgs::Odometry &start, nav_msgs::Odometry &end, Eigen::Quaterniond &result, double t)
bool average_quaternion(livox_mapping::GSOF &start, livox_mapping::GSOF &end, Eigen::Quaterniond &result, double t)
{
    try{
        if(t>1.0 || t<0.0)
        {
            return false;
        }
        //double starting[4] = {start.pose.pose.orientation.w, start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z};
        //double ending[4] = {end.pose.pose.orientation.w, end.pose.pose.orientation.x, end.pose.pose.orientation.y, end.pose.pose.orientation.z};
        double starting[4] = {start.orientation.w, start.orientation.x, start.orientation.y, start.orientation.z};
        double ending[4] = {end.orientation.w, end.orientation.x, end.orientation.y, end.orientation.z};
        double cosa = starting[0]*ending[0] + starting[1]*ending[1] + starting[2]*ending[2] + starting[3]*ending[3];
        if(cosa < 0.0)
        {
            ending[0] = -ending[0];
            ending[1] = -ending[1];
            ending[2] = -ending[2];
            ending[3] = -ending[3];
            cosa = -cosa;
        }
        
        double k0, k1;
        if(cosa > 0.9995)
        {
            k0 = 1.0 - t;
            k1 = t;
        }
        else
        {
            double sina = sqrt(1.0 - cosa*cosa);
            double a = atan2(sina, cosa);
            k0 = sin((1.0-t)*a) / sina;
            k1 = sin(t*a) / sina;
        }
        
        result.w() = starting[0]*k0 + ending[0]*k1;
        result.x() = starting[1]*k0 + ending[1]*k1;
        result.y() = starting[2]*k0 + ending[2]*k1;
        result.z() = starting[3]*k0 + ending[3]*k1;
        return true;
    }catch(exception& ex){
        cout << "\n *********************average_quaternion************************* \n" 
        << ex.what() 
        << "\n ************************average_quaternion********************** \n" 
        << endl;
        return false;
    }
}
// Mercator projection
bool mercator_proj(double B_0, double L0, double B, double L, double &X, double&Y)
{
    try{
        static double _A = 6378137, _B = 6356752.3142, _B0 = B_0, _L0 = L0;//_B0 = 22 * M_PI / 180, _L0 = 0;
        static double e = sqrt(1 - (_B/_A)*(_B/_A));
        static double e_ = sqrt((_A/_B)*(_A/_B) - 1);
        static double NB0 = ((_A*_A)/_B) / sqrt(1+e_*e_*cos(_B0)*cos(_B0));
        static double K = NB0 * cos(_B0);
        static double E = exp(1);
        
        if(L<-M_PI || L>M_PI || B<-M_PI_2 || B>M_PI_2)
        {
            return false;
        }
        
        Y = K * (L - _L0);
        X = K * log(tan(M_PI_4+B/2) * pow((1-e*sin(B))/(1+e*sin(B)), e/2));
        
        return true;
    }catch(exception& ex){
        cout << "\n **********************mercator_proj************************ \n" 
        << ex.what() 
        << "\n ************************mercator_proj********************** \n" 
        << endl;
        return false;
    }
}



