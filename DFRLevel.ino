/*Include the libraries we need*/
#include "RTClib.h" //Needed for communication with Real Time Clock
#include <SPI.h>//Needed for working with SD card
#include <SD.h>//Needed for working with SD card
#include <ArduinoLowPower.h>//Needed for putting Feather M0 to sleep between samples
#include <IridiumSBD.h>//Needed for communication with IRIDIUM modem 
#include <CSV_Parser.h>//Needed for parsing CSV data
#include <OneWire.h>//Needed for communication with DS18B20
#include <DallasTemperature.h>//Needed for communication with DS18B20
#include <QuickStats.h>//Needed for computing median 

/*Define global constants*/
const byte h2o_level_pin = A1;//Pin for taking analog reading of DFR current-to-voltage converter
const float range = 5000.0; // Depth measuring range 5000mm (for water)
const float density_water = 1.00;  // Pure water density assumed to be 1
const float vref = 3300.0; //Reference voltage in mV of MCU Vcc
const float init_current = 4.00; //Inital current in mA of probe out of water (from DFR datasheet)
const byte led = 13; // Built in led pin
const byte chip_select = 4; // For SD card
const byte irid_pwr_pin = 11; // Pwr pin to Iridium modem
const byte hyd_set_pin = 5; //Pwr set pin to DFR probe
const byte hyd_unset_pin = 6; //Pwr unset pin to DFR probe
const byte temp_pin = 12;//Data pin for DS18B20
const byte temp_pwr_pin = 9;//Pwr pin for DS18B20


/*Define global vars */
char **filename; //Name of log file
char **start_time;//Time at which logging begins
String filestr; //Filename as string
int16_t *sample_intvl; //Sample interval in seconds
int16_t *irid_freq;//Iridium transmit freqency in hours
uint32_t irid_freq_hrs;///Iridium transmit freqency in hours
uint32_t sleep_time;//Logger sleep time in milliseconds
uint32_t sample_n_;//Sample interval in seconds 
DateTime transmit_time;//Datetime varible for keeping IRIDIUM transmit time
DateTime present_time;//Var for keeping the current time
int err; //IRIDIUM status var
float data_current, depth; //Vars for computing water depth from analog voltage
int16_t *sample_n;//Number of water depth samples to take median of per reading


/*Define Iridium seriel communication COM1*/
#define IridiumSerial Serial1

/*Create library instances*/
RTC_PCF8523 rtc; // Setup a PCF8523 Real Time Clock instance
File dataFile; // Setup a log file instance
IridiumSBD modem(IridiumSerial); // Declare the IridiumSBD object

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(temp_pin);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

QuickStats stats;

/*Function reads data from a .csv logfile, and uses Iridium modem to send all observations
   since the previous transmission over satellite at midnight on the RTC.
*/
int send_hourly_data()
{

  // For capturing Iridium errors
  int err;

  // Provide power to Iridium Modem
  digitalWrite(irid_pwr_pin, HIGH);
  // Allow warm up
  delay(200);


  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  if (err != ISBD_SUCCESS)
  {
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
  }



  // Set paramters for parsing the log file
  CSV_Parser cp("sdf", true, ',');

  // Varibles for holding data fields
  char **datetimes;
  int16_t *h2o_depths;
  float *h2o_temps;


  // Read HOURLY.CSV file
  cp.readSDfile("/HOURLY.CSV");

  int num_rows = cp.getRowsCount();

  //Populate data arrays from logfile
  datetimes = (char**)cp["datetime"];
  h2o_depths = (int16_t*)cp["h2o_depth_mm"];
  h2o_temps = (float*)cp["h2o_temp_deg_c"];

  //Binary bufffer for iridium transmission (max allowed buffer size 340 bytes)
  uint8_t dt_buffer[340];

  //Buffer index counter var
  int buff_idx = 0;

  //Formatted for CGI script >> sensor_letter_code:date_of_first_obs:hour_of_first_obs:data
  String datestamp = "AB:" + String(datetimes[0]).substring(0, 10) + ":" + String(datetimes[0]).substring(11, 13) + ":";

  //Populate buffer with datestamp
  for (int i = 0; i < datestamp.length(); i++)
  {
    dt_buffer[buff_idx] = datestamp.charAt(i);
    buff_idx++;
  }

  //Get start and end date information from HOURLY.CSV time series data
  int start_year = String(datetimes[0]).substring(0, 4).toInt();
  int start_month = String(datetimes[0]).substring(5, 7).toInt();
  int start_day = String(datetimes[0]).substring(8, 10).toInt();
  int start_hour = String(datetimes[0]).substring(11, 13).toInt();
  int end_year = String(datetimes[num_rows - 1]).substring(0, 4).toInt();
  int end_month = String(datetimes[num_rows - 1]).substring(5, 7).toInt();
  int end_day = String(datetimes[num_rows - 1]).substring(8, 10).toInt();
  int end_hour = String(datetimes[num_rows - 1]).substring(11, 13).toInt();

  //Set the start time to rounded first datetime hour in CSV
  DateTime start_dt = DateTime(start_year, start_month, start_day, start_hour, 0, 0);
  //Set the end time to end of last datetime hour in CSV
  DateTime end_dt = DateTime(end_year, end_month, end_day, end_hour + 1, 0, 0);
  //For keeping track of the datetime at the end of each hourly interval
  DateTime intvl_dt;

  while (start_dt < end_dt)
  {

    intvl_dt = start_dt + TimeSpan(0, 1, 0, 0);

    //Declare average vars for each HYDROS21 output
    float mean_depth = -9999.0;
    float mean_temp = -9999.0;
    boolean is_first_obs = false;
    int N = 0;

    //For each observation in the HOURLY.CSV
    for (int i = 0; i < num_rows; i++) {

      //Read the datetime and hour
      String datetime = String(datetimes[i]);
      int dt_year = datetime.substring(0, 4).toInt();
      int dt_month = datetime.substring(5, 7).toInt();
      int dt_day = datetime.substring(8, 10).toInt();
      int dt_hour = datetime.substring(11, 13).toInt();
      int dt_min = datetime.substring(14, 16).toInt();
      int dt_sec = datetime.substring(17, 19).toInt();

      DateTime obs_dt = DateTime(dt_year, dt_month, dt_day, dt_hour, dt_min, dt_sec);

      //Check in the current observatioin falls withing time window
      if (obs_dt >= start_dt && obs_dt <= intvl_dt)
      {

        //Get data
        float h2o_depth = (float) h2o_depths[i];
        float h2o_temp = h2o_temps[i];

        //Check if this is the first observation for the hour
        if (is_first_obs == false)
        {
          //Update average vars
          mean_depth = h2o_depth;
          mean_temp = h2o_temp;
          is_first_obs = true;
          N++;
        } else {
          //Update average vars
          mean_depth = mean_depth + h2o_depth;
          mean_temp = mean_temp + h2o_temp;
          N++;
        }

      }
    }

    //Check if there were any observations for the hour
    if (N > 0)
    {
      //Compute averages
      mean_depth = (mean_depth / (float) N);
      mean_temp = (mean_temp / (float) N) * 10.0;


      //Assemble the data string
      String datastring = String(round(mean_depth)) + "," + String(round(mean_temp)) + ':';


      //Populate the buffer with the datastring
      for (int i = 0; i < datastring.length(); i++)
      {
        dt_buffer[buff_idx] = datastring.charAt(i);
        buff_idx++;
      }

    }

    start_dt = intvl_dt;

  }

  // Prevent from trying to charge to quickly, low current setup
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation, blink led (1-sec) if there was an issue
  err = modem.begin();

  if (err == ISBD_IS_ASLEEP)
  {
    modem.begin();
  }

  //Indicate the modem is trying to send with led
  digitalWrite(led, HIGH);

  //transmit binary buffer data via iridium
  err = modem.sendSBDBinary(dt_buffer, buff_idx);

  //If transmission failed and message is not too large try once more, increase time out
  if (err != ISBD_SUCCESS && err != 13)
  {
    err = modem.begin();
    modem.adjustSendReceiveTimeout(500);
    err = modem.sendSBDBinary(dt_buffer, buff_idx);

  }

  digitalWrite(led, LOW);


  //Kill power to Iridium Modem by writing the base pin low on PN2222 transistor
  digitalWrite(irid_pwr_pin, LOW);
  delay(30);


  //Remove previous daily values CSV as long as send was succesfull, or if message is more than 340 bites

  SD.remove("/HOURLY.CSV");

  return err;


}


//Function for obtaining mean water level from n sensor readings of DFRobot level sensor
int avgWaterLevl(uint32_t n)
{
  //Switch 12V power to level sensor on
  digitalWrite(hyd_set_pin, HIGH);
  delay(30);
  digitalWrite(hyd_set_pin, LOW);

  //Allow warm up
  delay(2000);

  float values[n];

  for (int i = 0; i < n; i++)
  {

    float level_value = (float) analogRead(h2o_level_pin);

    //Read voltage output of H2O level sensor
    float level_voltage =  level_value * (vref / 4095.0);

    //Convert to current
    float level_current = level_voltage / 120.0; //Sense Resistor:120ohm

    values[i] = level_current;

    delay(10);

  }

  float median_current = stats.median(values, n);

  //Calculate water depth (mm) from current readings (see datasheet)
  float depth = (median_current - init_current) * (range / density_water / 16.0);

  //Switch power to level sensor off
  digitalWrite(hyd_unset_pin, HIGH);
  delay(30);
  digitalWrite(hyd_unset_pin, LOW);


  return round(depth);

}


/*
   The setup function. We only start the sensors, RTC and SD here
*/
void setup(void)
{
  // Set pin modes
  pinMode(irid_pwr_pin, OUTPUT);
  digitalWrite(irid_pwr_pin, LOW);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(hyd_set_pin, OUTPUT);
  digitalWrite(hyd_set_pin, LOW);
  pinMode(hyd_unset_pin, OUTPUT);
  digitalWrite(hyd_unset_pin, HIGH);
  delay(30);
  digitalWrite(hyd_unset_pin, LOW);
  pinMode(temp_pin, INPUT);
  pinMode(h2o_level_pin, INPUT);
  pinMode(temp_pwr_pin, OUTPUT);

  analogReadResolution(12);


  //Make sure a SD is available (1-sec flash led means SD card did not initialize)
  while (!SD.begin(chip_select)) {
    digitalWrite(led, HIGH);
    delay(2000);
    digitalWrite(led, LOW);
    delay(2000);
  }

  //Set paramters for parsing the log file
  CSV_Parser cp("sddsd", true, ',');


  //Read IRID.CSV
  while (!cp.readSDfile("/PARAM.txt"))
  {
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
  }


  //Populate data arrays from logfile
  filename = (char**)cp["filename"];
  sample_intvl = (int16_t*)cp["sample_intvl"];
  irid_freq = (int16_t*)cp["irid_freq"];
  start_time = (char**)cp["start_time"];
  sample_n = (int16_t*)cp["sample_n"];


  //Sleep time between samples in seconds
  sleep_time = sample_intvl[0] * 1000;

  //Log file name
  filestr = String(filename[0]);

  //Iridium transmission frequency
  irid_freq_hrs = irid_freq[0];

  sample_n_ = sample_n[0];

  //Get logging start time from parameter file
  int start_hour = String(start_time[0]).substring(0, 3).toInt();
  int start_minute = String(start_time[0]).substring(3, 5).toInt();
  int start_second = String(start_time[0]).substring(6, 8).toInt();

  // Make sure RTC is available
  while (!rtc.begin())
  {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
  }

  // Start up the DallasTemp library

  sensors.begin();

  present_time = rtc.now();
  transmit_time = DateTime(present_time.year(),
                           present_time.month(),
                           present_time.day(),
                           start_hour + irid_freq_hrs,
                           start_minute,
                           start_second);




}

/*
   Main function, sample HYDROS21 and sample interval, log to SD, and transmit hourly averages over IRIDIUM at midnight on the RTC
*/
void loop(void)
{

  //Get the present datetime
  present_time = rtc.now();

  //If the presnet time has reached transmit_time send all data since last transmission averaged hourly
  if (present_time >= transmit_time)
  {
    int send_status = send_hourly_data();

    //Update next Iridium transmit time by 'irid_freq_hrs'
    transmit_time = (transmit_time + TimeSpan(0, irid_freq_hrs, 0, 0));

  }

  //Ping DS18B20
  digitalWrite(temp_pwr_pin, HIGH);
  delay(100);
  sensors.begin();
  delay(750);
  sensors.requestTemperatures();
  float temp_c = sensors.getTempCByIndex(0);
  digitalWrite(temp_pwr_pin, LOW);

  int h2o_level = avgWaterLevl(sample_n_);

  //Sample the HYDROS21 sensor for a reading
  String datastring = present_time.timestamp() + "," + String(h2o_level) + "," + String(temp_c);


  //Write header if first time writing to the logfile
  if (!SD.exists(filestr.c_str()))
  {
    dataFile = SD.open(filestr.c_str(), FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("datetime,h2o_depth_mm,h2o_temp_deg_c");
      dataFile.close();
    }

  } else {
    //Write datastring and close logfile on SD card
    dataFile = SD.open(filestr.c_str(), FILE_WRITE);
    if (dataFile)
    {
      dataFile.println(datastring);
      dataFile.close();
    }

  }


  /*The HOURLY.CSV file is the same as the log-file, but only contains observations since the last transmission and is used by the send_hourly_data() function */

  //Write header if first time writing to the DAILY file
  if (!SD.exists("HOURLY.CSV"))
  {
    //Write datastring and close logfile on SD card
    dataFile = SD.open("HOURLY.CSV", FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("datetime,h2o_depth_mm,h2o_temp_deg_c");
      dataFile.close();
    }
  } else {
    //Write datastring and close logfile on SD card
    dataFile = SD.open("HOURLY.CSV", FILE_WRITE);
    if (dataFile)
    {
      dataFile.println(datastring);
      dataFile.close();
    }
  }

  //Flash led to idicate a sample was just taken
  digitalWrite(led, HIGH);
  delay(250);
  digitalWrite(led, LOW);
  delay(250);

  //Put logger in low power mode for lenght 'sleep_time'
  LowPower.sleep(sleep_time);


}
