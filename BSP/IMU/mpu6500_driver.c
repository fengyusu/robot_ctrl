#include "mpu6500_driver.h"
#include "delay.h"


extern uint8_t isMPU6500_is_DRY;   // mpu6500 interrupt中断标志

volatile MPU6500_RAW_DATA    MPU6500_Raw_Data;    //原始数据
volatile MPU6500_REAL_DATA   MPU6500_Real_Data;

uint8_t mpu_buf[20]={0};       //save the data of acc gyro & mag using SPI
int16_t MPU6500_FIFO[7][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
MagMaxMinData_t MagMaxMinData;
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
int16_t MPU6500_Lastax,MPU6500_Lastay,MPU6500_Lastaz,MPU6500_LastTem,MPU6500_Lastgx,MPU6500_Lastgy,MPU6500_Lastgz;

/**
* @fn       MPU6500_RWAddr
* @brief    mpu6500读写字节
* @param    addr：读写地址
*           data：写入的字节
* @retval   读取到的字节
* @note     无
*/
static uint8_t MPU6500_RWAddr(uint8_t addr, uint8_t data)
{
    u8 tmp = 0;
    NSS_RESET();
    SPI5_ReadWriteByte(addr);
    tmp = SPI5_ReadWriteByte(data);
    NSS_SET();
    return tmp;    
}
/**
* @fn       MPU6500_ReadData
* @brief    mpu6500读数据
* @param    addr：读写地址
*           pdat：存数据的首地址
            length：读取的字节数
* @retval   读取状态
* @note     无
*/
static void MPU6500_ReadData(uint8_t addr, uint8_t* pdat, uint16_t length)
{
    addr = READ|addr;
    SPI5_ReadData(addr, pdat, length);   
}
/**
* @fn       MPU6500_ReadData
* @brief    mpu6500写数据
* @param    addr：读写地址
*           pdat：数据的首地址
            length：写入的字节数
* @retval   写状态
* @note     无
*/
//static void MPU6500_WriteData(uint8_t addr, uint8_t* pdat, uint16_t length)
//{
//    addr = WRITE|addr;
//    SPI5_ReadData(addr, pdat, length);
//}

/**
* @fn       MPU6500_I2C_SLV1_SLV4_Config
* @brief    Initialize the MPU6500 I2C Slave 1 for I2C writing and I2C Slave 4 for I2C one byte reading.
* @param    Slave device address, Address[6:0]
* @retval   void
* @note     
*/
static void MPU6500_I2C_SLV1_SLV4_Config(uint8_t device_address)
{
    //配置slv1 写
	MPU6500_RWAddr(WRITE | I2C_SLV1_ADDR, device_address);  
	delay_ms(DELAY_CNT);
	
	//配置slv4 读
	MPU6500_RWAddr(WRITE | I2C_SLV4_ADDR, READ|device_address );  
	delay_ms(DELAY_CNT);
}
/**
* @fn       IST8310_Reg_Write_By_MPU6500
* @brief    Write IST8310 register through MPU6500's I2C Master
* @param    Register address￡oreg_address, Register content: reg_data
* @retval   void
* @note     MPU6500_I2C_SLV1_SLV4_Config need to be called before.
*/
static void IST8310_Reg_Write_By_MPU6500(uint8_t reg_address, uint8_t reg_data)
{
	//turn off slave 1 at first 
	MPU6500_RWAddr(WRITE | I2C_SLV1_CTRL, 0x00);	
	delay_ms(2);
    MPU6500_RWAddr(WRITE | I2C_SLV1_REG, reg_address);
	delay_ms(2);
	MPU6500_RWAddr(WRITE | I2C_SLV1_DO, reg_data);
	delay_ms(2);
	//turn on slave 1 with one byte transmitting
    MPU6500_RWAddr(WRITE | I2C_SLV1_CTRL, 0x80 | 0x01);	
	//wait longer to ensure the data is transmitted from slave 1
	delay_ms(10);
}
/**
* @fn       IST8310_Reg_Read_By_MPU6500
* @brief    Write IST8310 register through MPU6500's I2C Master
* @param    Register address￡oreg_address
* @retval   void
* @note     MPU6500_I2C_SLV1_SLV4_Config need to be called before.
*/
static uint8_t IST8310_Reg_Read_By_MPU6500(uint8_t reg_address)
{
	uint8_t retval;
	MPU6500_RWAddr(WRITE | I2C_SLV4_REG, reg_address);
	delay_ms(10);
	MPU6500_RWAddr(WRITE | I2C_SLV4_CTRL, 0x80);
	delay_ms(10);
	retval = MPU6500_RWAddr(READ | I2C_SLV4_DI, 0xFF);
	//turn off slave4 after read
	MPU6500_RWAddr(WRITE | I2C_SLV4_CTRL, 0x00);
	delay_ms(10);
	return retval;
}

/**
* @fn       MPU6500_I2C_SLV1_SLV4_Disable
* @brief    Disable slave 1 and slave 4.
* @param    void
* @retval   void
* @note     
*/
static void MPU6500_I2C_SLV1_SLV4_Disable(void)
{
	//turn off slave1 and slave4
	MPU6500_RWAddr(WRITE | I2C_SLV1_CTRL, 0x00);
	delay_ms(2);
	MPU6500_RWAddr(WRITE | I2C_SLV4_CTRL, 0x00);
	delay_ms(2);
}

/**
* @fn       MPU6500_I2C_AutoReadConfig
* @brief    Initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    Slave device address, Address[6:0]
* @retval   void
* @note     
*/
static void MPU6500_I2C_AutoReadConfig(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    //configure the device address of the IST8310
    //use slave1,auto transmit single measure mode.
	MPU6500_RWAddr(WRITE | I2C_SLV1_ADDR, device_address );
	delay_ms(2);
	MPU6500_RWAddr(WRITE | I2C_SLV1_REG, IST8310_R_CONFA);
    delay_ms(2);
    MPU6500_RWAddr(WRITE | I2C_SLV1_DO, IST8310_ODR_MODE);   
	delay_ms(2);   
    
    //use slave0,auto read data
	MPU6500_RWAddr(WRITE | I2C_SLV0_ADDR, 0x80|device_address );
	delay_ms(2);
	MPU6500_RWAddr(WRITE | I2C_SLV0_REG, reg_base_addr);
    delay_ms(2);
    
	//every eight mpu6500 internal samples one i2c master read
	MPU6500_RWAddr(WRITE | I2C_SLV4_CTRL, 0x03);   
	delay_ms(2);    
	//enable slave 0 and 1 access delay 
	MPU6500_RWAddr(WRITE | I2C_MST_DELAY_CTRL, 0x01|0x02);  
	delay_ms(2);   
	//enable slave 1 auto transmit
    MPU6500_RWAddr(WRITE | I2C_SLV1_CTRL, 0x80 | 0x01);  
	delay_ms(6);                            //Wait 6ms (minimum waiting time for 16 times internal average setup)
	//enable slave 0 with data_num bytes reading 
    MPU6500_RWAddr(WRITE | I2C_SLV0_CTRL, 0x80 | data_num);   
	delay_ms(2);
}

/**
* @fn       BSP_IST8310L_Init
* @brief    IST8310初始化
* @param    无
* @retval   0：初始化成功  其他：失败
* @note     
*/
uint8_t BSP_IST8310_Init(void)
{
	MPU6500_RWAddr(WRITE | USER_CTRL, 0x30);       //使能I2C主模式
	delay_ms(10);	
	MPU6500_RWAddr(WRITE | I2C_MST_CTRL, 0x0D);    //400Khz I2C 
	delay_ms(10);
	
    //turn on slave 1 and slave 4 
	MPU6500_I2C_SLV1_SLV4_Config(IST8310_ADDRESS);
    /*配置寄存器IST8310_R_CONFB*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_R_CONFB, 0x01);	//软件复位
	delay_ms(10);     
	if(IST8310_Reg_Read_By_MPU6500(IST8310_WHO_AM_I) != IST8310_DEVICE_ID_A) return 1; 
	    
    /*配置寄存器IST8310_R_CONFB*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_R_CONFB, 0x01);	//软件复位
	delay_ms(10); 
    /*配置寄存器IST8310_R_CONFA*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_R_CONFA, 0x00);    //先配置为准备模式，访问寄存器
	if(IST8310_Reg_Read_By_MPU6500(IST8310_R_CONFA) != 0x00) return 1;	
	delay_ms(10);
    /*配置寄存器IST8310_R_CONFB*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_R_CONFB, 0x00);	//正常工作，不开中断
	if(IST8310_Reg_Read_By_MPU6500(IST8310_R_CONFB) != 0x00) return 1;	
	delay_ms(10);
	
//	/*配置寄存器IST8310_R_CONFB*/	
//	IST8310_Reg_Write_By_MPU6500(IST8310_R_CONFB, 0x0C);	//使能数据准备引脚
//	if(IST8310_Reg_Read_By_MPU6500(IST8310_R_CONFB) != 0x0C) return 1;	
//	delay_ms(10);    
    
    /*配置寄存器IST8310_R_CONFA*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_AVGCNTL, 0x24);    //低噪声模式，y轴16次取平均，x和z轴16次取平均
	if(IST8310_Reg_Read_By_MPU6500(IST8310_AVGCNTL) != 0x24) return 1;	
	delay_ms(10);
    /*配置寄存器IST8310_R_CONFA*/	
	IST8310_Reg_Write_By_MPU6500(IST8310_PDCNTL, 0xC0);    //Set/Reset pulse duration setup,normal mode
	if(IST8310_Reg_Read_By_MPU6500(IST8310_PDCNTL) != 0xC0) return 1;	
	delay_ms(10);    
	
    //turn off slave 1 and slave 4
	MPU6500_I2C_SLV1_SLV4_Disable();	
    //configure and turn on slave 0 
	MPU6500_I2C_AutoReadConfig(IST8310_ADDRESS, IST8310_R_XL, 0x06);  
	
	return 0;
}

uint8_t MPU6500_Init(void)
{
//    uint8_t retry_cnt = 0;
    
    MPU6500_SpiConfig();
    MPU6500_IntConfig();
    delay_ms(DELAY_CNT);
    
    if(MPU6500_RWAddr(READ|WHO_AM_I, 0xFF) != 0x70)  return 1;
    delay_ms(DELAY_CNT);
    
    MPU6500_RWAddr(WRITE|PWR_MGMT_1, 0x80);	    //电源管理,复位MPU6500
    delay_ms(DELAY_CNT);
    
    MPU6500_RWAddr(WRITE|PWR_MGMT_1, 0x00);	    //解除休眠状态
    delay_ms(DELAY_CNT);    
    if(MPU6500_RWAddr(READ|PWR_MGMT_1, 0xFF) != 0x00)  return 1;
    delay_ms(DELAY_CNT);
    
    MPU6500_RWAddr(WRITE|CONFIG, 0x03);         //Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz
	delay_ms(DELAY_CNT);                        //gyro bandwidth 41Hz
    if(MPU6500_RWAddr(READ|CONFIG, 0xFF) != 0x03)  return 1;
    delay_ms(DELAY_CNT); 
    
    MPU6500_RWAddr(WRITE|GYRO_CONFIG, 0x10);    //1000degre/s //0x00:+/-250   0x08:+/-500  0x10:+/-1000  0x18:+/-2000
	delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|GYRO_CONFIG, 0xFF) != 0x10)  return 1;
    delay_ms(DELAY_CNT); 
    
    MPU6500_RWAddr(WRITE|ACCEL_CONFIG, 0x10);   //0x00:+/-2G   0x08:+/-4G  0x10:+/-8G   0x18:+/-16G
	delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|ACCEL_CONFIG, 0xFF) != 0x10)  return 1;
    delay_ms(DELAY_CNT); 
    
	MPU6500_RWAddr(WRITE|ACCEL_CONFIG2, 0x03);  //41Hz bw
	delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|ACCEL_CONFIG2, 0xFF) != 0x03)  return 1;
    delay_ms(DELAY_CNT); 
    
    MPU6500_RWAddr(WRITE|INT_PIN_CFG, 0x00);   //logic level for the INT pin is active high
	delay_ms(DELAY_CNT);                       //the INT pin emits a 50us long pulse, not latched    bypass mode disabled
    if(MPU6500_RWAddr(READ|INT_PIN_CFG, 0xFF) != 0x00)  return 1;
    delay_ms(DELAY_CNT); 
    
    MPU6500_RWAddr(WRITE|INT_ENABLE, 0x00);   //暂时禁止数据准备好中断
    delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|INT_ENABLE, 0xFF) != 0x00)  return 1;
    delay_ms(DELAY_CNT); 
	
    MPU6500_RWAddr(WRITE|USER_CTRL,0x00);   //设置mpu6500 IIC masters mode  disabled 暂时不让mpu6500控制aux IIC接口
    delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|USER_CTRL, 0xFF) != 0x00)  return 1;
    delay_ms(DELAY_CNT); 
    
    delay_ms(500);
    return 0;     
}

uint8_t MPU6500_Enable(void)
{
    MPU6500_RWAddr(WRITE|SMPLRT_DIV, 0x01);  //Sample Rate: Gyro output rate / (1 + 1) = 500Hz
	delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|SMPLRT_DIV, 0xFF) != 0x01)  return 1;
    delay_ms(DELAY_CNT); 
    
    MPU6500_RWAddr(WRITE|INT_ENABLE, 0x01);   //enable data ready interrupt 
    delay_ms(DELAY_CNT);
    if(MPU6500_RWAddr(READ|INT_ENABLE, 0xFF) != 0x01)  return 1;
    delay_ms(DELAY_CNT); 
    
    return 0;    
}
/**********************************************************************************/
/*将MPU6500_ax,MPU6500_ay, MPU6500_az,MPU6500_gx, MPU6500_gy, MPU6500_gz处理后存储*/
/**********************************************************************************/
void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t tem) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	int32_t sum=0;
	
	for(uint8_t i=1;i<10;i++)
	{
		MPU6500_FIFO[0][i-1]=MPU6500_FIFO[0][i];
		MPU6500_FIFO[1][i-1]=MPU6500_FIFO[1][i];
		MPU6500_FIFO[2][i-1]=MPU6500_FIFO[2][i];
		MPU6500_FIFO[3][i-1]=MPU6500_FIFO[3][i];
		MPU6500_FIFO[4][i-1]=MPU6500_FIFO[4][i];
		MPU6500_FIFO[5][i-1]=MPU6500_FIFO[5][i];
        MPU6500_FIFO[6][i-1]=MPU6500_FIFO[6][i];
	}
	
	MPU6500_FIFO[0][9]=ax;//将新的数据放置到数据的最后面
	MPU6500_FIFO[1][9]=ay;
	MPU6500_FIFO[2][9]=az;
	MPU6500_FIFO[3][9]=gx;
	MPU6500_FIFO[4][9]=gy;
	MPU6500_FIFO[5][9]=gz;
    MPU6500_FIFO[6][9]=tem;
	
    for(uint8_t i = 0;i<7;i++)
    {
        sum = 0;
        for(uint8_t j = 0;j<10;j++)
        {
            sum+=MPU6500_FIFO[i][j];           
        }
        MPU6500_FIFO[i][10]=sum/10;
    }	
}

/**************************实现函数********************************************
*函数原型:		void MPU6500_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tem) 
*功　　能:	    读取 MPU6500的当前测量值
*******************************************************************************/
void MPU6500_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tem) 
{  
	if(isMPU6500_is_DRY)
	{
		isMPU6500_is_DRY = 0;
		MPU6500_ReadData(MPU6500_DATA_START,mpu_buf,14);  //
//		IST8310_ReadData(&(mpu_buf[14]));  //14-19为磁力计数据
        //加计
		MPU6500_Lastax=(((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
		MPU6500_Lastay=(((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
		MPU6500_Lastaz=(((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];
		//温度
        MPU6500_LastTem=(((int16_t)mpu_buf[6]) << 8) | mpu_buf[7];
        //陀螺
		MPU6500_Lastgx=(((int16_t)mpu_buf[8]) << 8) | mpu_buf[9];
		MPU6500_Lastgy=(((int16_t)mpu_buf[10]) << 8) | mpu_buf[11];
		MPU6500_Lastgz=(((int16_t)mpu_buf[12]) << 8) | mpu_buf[13];
			
		MPU6500_DataSave(MPU6500_Lastax,MPU6500_Lastay,MPU6500_Lastaz,MPU6500_Lastgx,MPU6500_Lastgy,MPU6500_Lastgz,MPU6500_LastTem);  		
		*ax  =MPU6500_FIFO[0][10];
		*ay  =MPU6500_FIFO[1][10];
		*az = MPU6500_FIFO[2][10];
		*gx  =MPU6500_FIFO[3][10] - 0;
		*gy = MPU6500_FIFO[4][10] - 0;
		*gz = MPU6500_FIFO[5][10] - 0;
        *tem = MPU6500_FIFO[6][10];
	} 
	else
	{       //读取上一次的值
		*ax = MPU6500_FIFO[0][10];
		*ay = MPU6500_FIFO[1][10];
		*az = MPU6500_FIFO[2][10];
		*gx = MPU6500_FIFO[3][10];
		*gy = MPU6500_FIFO[4][10];
		*gz = MPU6500_FIFO[5][10];
        *tem = MPU6500_FIFO[6][10];
	}
}
void MPU6500_getlastMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,int16_t* tem)
{
	*ax  =MPU6500_FIFO[0][10];
	*ay  =MPU6500_FIFO[1][10];
	*az = MPU6500_FIFO[2][10];
	*gx  =MPU6500_FIFO[3][10];
	*gy = MPU6500_FIFO[4][10];
	*gz = MPU6500_FIFO[5][10];
    *tem = MPU6500_FIFO[6][10];
}
/**************************实现函数********************************************
*函数原型:	  void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器读取5883寄存器的值
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void IST8310_ReadData(u8 *vbuff) 
{   
   MPU6500_ReadData(EXT_SENS_DATA_00,vbuff,6);   //读取到磁力计数据
}

/**************************实现函数********************************************
*函数原型:	   void  IST8310_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  IST8310_DataSave(int16_t x,int16_t y,int16_t z)
{
	int32_t sum=0;

	for(uint8_t i=1;i<10;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[0][9]= x;//将新的数据放置到 数据的最后面
	HMC5883_FIFO[1][9]= y;
	HMC5883_FIFO[2][9]= z;
	
    for(uint8_t i = 0;i<3;i++)
    {
        sum = 0;
        for(uint8_t j = 0;j<10;j++)
        {
            sum+=HMC5883_FIFO[i][j];           
        }
        HMC5883_FIFO[i][10]=sum/10;
    }	

	//以上全部为未校准数据,记录磁力计的最大最小值
	if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
	}
	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
	}

	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
	}
	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
	}		
}

/**************************实现函数********************************************
*函数原型:	  void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{
    IST8310_ReadData(&mpu_buf[14]);
    IST8310_DataSave((((int16_t)mpu_buf[15] << 8) | mpu_buf[14]), (((int16_t)mpu_buf[17] << 8) | mpu_buf[16]), -(((int16_t)mpu_buf[19] << 8) | mpu_buf[18]) );
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10];
    *z = HMC5883_FIFO[2][10];
}
/**************************实现函数********************************************
*函数原型:	  void IST8310_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void IST8310_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = HMC5883_FIFO[0][10];
    *y = HMC5883_FIFO[1][10]; 
    *z = HMC5883_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void IST8310_mgetValues(volatile float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
void IST8310_mgetValues(volatile float *arry) 
{
    int16_t xr,yr,zr;
    IST8310_getRaw(&xr, &yr, &zr);
    arry[0]= HMC5883_lastx=(float)xr;
    arry[1]= HMC5883_lasty=(float)yr;
    arry[2]= HMC5883_lastz=(float)zr;
}



/******************************** end of bsp_mpu6500_driver.c ***********************************/

