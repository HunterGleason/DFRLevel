/***********************************************************
  DFRobot Gravity: Analog Current to Voltage Converter(For 4~20mA Application)
  SKU:SEN0262

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

/*Import Libraries*/ 
#include <SPI.h>/*Needed for working with SD card*/
#include <SD.h>/*Needed for working with SD card*/
#include "RTClib.h"/*Needed for communication with Real Time Clock*/
#include <CSV_Parser.h>/*Needed for parsing CSV data*/
#include <IridiumSBD.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*Define global constants */
const byte H2O_LEVEL_PIN = A1;
const float RANGE = 5000.0; // Depth measuring range 5000mm (for water)
const float DENSITY_WATER = 1.00;  // Pure water density assumed to be 1
const byte SETPIN = 5;
const byte UNSETPIN = 6;
const byte DONEPIN = 12;
const int16_t sample_n = 30;
const byte LED = 13;
const byte TEMP_PIN = 9;
const byte TEMP_PWR = 10;
const byte CHIP_SELECT = 4;


/*Define global vars */
int16_t dataVoltage;
float dataCurrent, depth; //unit:mA
char **filename;/*Desired name for data file !!!must be less than equal to 8 char!!!*/
float *CURRENT_INIT;// Current @ 0mm (uint: mA)
int16_t *VREF;// ADC's reference voltage on your Adalogger,typical value:3300mV, measure for higher accuracy 

/*Initiate global libraries*/

#define IridiumSerial Serial1 /* Define Serial1 for communicating with Iridium Modem */

/* Define the logfile */
File dataFile;

/*Define PCF8523 RTC*/
RTC_PCF8523 rtc;

/*Declare the IridiumSBD object*/
IridiumSBD modem(IridiumSerial);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/*Define global functions*/

/*Function pings RTC for datetime and returns formated datestamp*/
String gen_date_str(DateTime now) {

  //Format current date time values for writing to SD
  String yr_str = String(now.year());
  String mnth_str;
  if (now.month() >= 10)
  {
    mnth_str = String(now.month());
  } else {
    mnth_str = "0" + String(now.month());
  }

  String day_str;
  if (now.day() >= 10)
  {
    day_str = String(now.day());
  } else {
    day_str = "0" + String(now.day());
  }

  String hr_str;
  if (now.hour() >= 10)
  {
    hr_str = String(now.hour());
  } else {
    hr_str = "0" + String(now.hour());
  }

  String min_str;
  if (now.minute() >= 10)
  {
    min_str = String(now.minute());
  } else {
    min_str = "0" + String(now.minute());
  }


  String sec_str;
  if (now.second() >= 10)
  {
    sec_str = String(now.second());
  } else {
    sec_str = "0" + String(now.second());
  }

  //Assemble a data string for logging to SD, with date-time, snow depth (mm), temperature (deg C) and humidity (%)
  String datestring = yr_str + "-" + mnth_str + "-" + day_str + " " + hr_str + ":" + min_str + ":" + sec_str;

  return datestring;
}

/*Function reads data from a daily logfile, and uses Iridium modem to send all observations
   for the previous day over satellite at midnight on the RTC.
*/
void send_daily_data(DateTime now)
{

  //Use IRID.CSV to keep track of day
  if (!SD.exists("IRID.CSV"))
  {
    dataFile = SD.open("IRID.CSV", FILE_WRITE);
    dataFile.println("day,day1");
    dataFile.println(String(now.day()) + "," + String(now.day()));
    dataFile.close();

  }

  //Set up parse params for IRID.CSV
  CSV_Parser cp(/*format*/ "s-", /*has_header*/ true, /*delimiter*/ ',');

  //Read IRID.CSV
  while (!cp.readSDfile("/IRID.CSV"))
  {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }

  //Get day from IRID CSV
  char **irid_day = (char**)cp["day"];

  //If IRID day matches RTC day
  if (String(irid_day[0]).toInt() == now.day())
  {
    //Update IRID.CSV with new day
    SD.remove("IRID.CSV");
    dataFile = SD.open("IRID.CSV", FILE_WRITE);
    dataFile.println("day,day1");
    DateTime next_day = (DateTime(now.year(), now.month(), now.day()) + TimeSpan(1, 0, 0, 0));
    dataFile.println(String(next_day.day()) + "," + String(next_day.day()));
    dataFile.close();

    //For capturing Iridium errors
    int err;


    // Start the serial port connected to the satellite modem
    IridiumSerial.begin(19200);

    // Begin satellite modem operation
    err = modem.begin();
    if (err != ISBD_SUCCESS)
    {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
    }


    //Set paramters for parsing the log file
    CSV_Parser cp("sdf", true, ',');

    //Varibles for holding data fields
    char **datetimes;
    int16_t *h2o_depths;
    float *h2o_temps;

    //Read IRID.CSV
    cp.readSDfile("/DAILY.csv");


    //Populate data arrays from logfile
    datetimes = (char**)cp["datetime"];
    h2o_depths = (int16_t*)cp["h2o_depth_mm"];
    h2o_temps = (float*)cp["h2o_temp_deg_c"];

    //Binary bufffer for iridium transmission (max allowed buffer size 340 bytes)
    uint8_t dt_buffer[340];
    int buff_idx = 0;

    //Get the start datetime stamp as string
    String datestamp = String(datetimes[0]).substring(0, 10);

    for (int i = 0; i < datestamp.length(); i++)
    {
      dt_buffer[buff_idx] = datestamp.charAt(buff_idx);
      buff_idx++;
    }

    dt_buffer[buff_idx] = ':';
    buff_idx++;


    for (int day_hour = 0; day_hour < 24; day_hour++)
    {

      float mean_depth = 999.0;
      float mean_temp = 999.0;
      boolean is_obs = false;
      int N = 0;

      //For each observation in the CSV
      for (int i = 0; i < cp.getRowsCount(); i++) {

        String datetime = String(datetimes[i]);
        int dt_hour = datetime.substring(11, 13).toInt();

        if (dt_hour == day_hour)
        {


          float h2o_depth = (float) h2o_depths[i];
          float h2o_temp = h2o_temps[i];

          if (is_obs == false)
          {
            mean_depth = h2o_depth;
            mean_temp = h2o_temp;
            is_obs = true;
            N++;
          } else {
            mean_depth = mean_depth + h2o_depth;
            mean_temp = mean_temp + h2o_temp;
            N++;
          }

        }
      }

      if (N > 0)
      {
        mean_depth = mean_depth / N;
        mean_temp = (mean_temp / N) * 10.0;
      }

      //String datastring = String(round(mean_depth)) + ',' + String(round(mean_temp)) + ',' + String(round(mean_ec)) + ':';
      String datastring = String(round(mean_depth)) + ',' + String(round(mean_temp)) + ':';

      for (int i = 0; i < datastring.length(); i++)
      {
        dt_buffer[buff_idx] = datastring.charAt(i);
        buff_idx++;
      }

    }

    digitalWrite(LED, HIGH);
    //transmit binary buffer data via iridium
    err = modem.sendSBDBinary(dt_buffer, buff_idx);
    digitalWrite(LED, LOW);

    if (err != ISBD_SUCCESS)
    {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
    }


    //Remove previous daily values CSV
    SD.remove("DAILY.csv");
  }
}


//Function for obtaining mean water level from n sensor readings of DFRobot level sensor
int avgWaterLevl(int n)
{
  //Switch 12V power to level sensor on 
  digitalWrite(SETPIN, HIGH);
  delay(30);
  digitalWrite(SETPIN,LOW);
  
  float avg_current = 0.0;

  for (int i = 0; i < n; i++)
  {
    //Read voltage output of H2O level sensor
    float level_voltage =  analogRead(H2O_LEVEL_PIN) * (VREF[0] / 4095.0);

    //Convert to current
    float level_current = level_voltage / 120.0; //Sense Resistor:120ohm

    avg_current = avg_current + level_current;

    delay(100);

  }

  avg_current = avg_current / (float)n;

  //Calculate water depth (mm) from current readings (see datasheet)
  float depth = (avg_current - CURRENT_INIT[0]) * (RANGE / DENSITY_WATER / 16.0);

  //Switch power to level sensor off 
  digitalWrite(UNSETPIN, HIGH);
  delay(30);
  digitalWrite(UNSETPIN,LOW);


  return round(depth);

  

}


void setup() {

  modem.sleep();

  //Set pin modes for digital IO
  pinMode(DONEPIN, OUTPUT);
  pinMode(SETPIN, OUTPUT);
  pinMode(UNSETPIN, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(TEMP_PWR,OUTPUT);
  pinMode(H2O_LEVEL_PIN,INPUT);

  //Set ADC resolution to 12-bit 
  analogReadResolution(12);


  //Make sure a SD is available (1-sec flash LED means SD card did not initialize)
  while (!SD.begin(CHIP_SELECT))
  {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }

  // Start RTC (10-sec flash LED means RTC did not initialize)
  while (!rtc.begin())
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }

  //Set paramters for parsing the parameter file
  CSV_Parser cp(/*format*/ "sdf", /*has_header*/ true, /*delimiter*/ ',');

  //Read the parameter file off SD card (HYDROS.CSV), 1/4-sec flash means file is not available
  while (!cp.readSDfile("/HYDROS.CSV"))
  {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }

  //Read values from SNOW_PARAM.TXT into global varibles
  filename = (char**)cp["filename"];
  CURRENT_INIT = (float*)cp["initial_current"];
  VREF = (int16_t*)cp["refrence_voltage"];

  //Write header if first time writing to the file
  if (!SD.exists(filename[0]))
  {
    dataFile = SD.open(filename[0], FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("datetime,h2o_depth_mm,h2o_temp_deg_c");
      dataFile.close();
    }

  }

  //Get datatime
  DateTime now = rtc.now();
  String datastring = gen_date_str(now);

  //Read water level 
  int waterLevel = avgWaterLevl(sample_n);

  //Read water temperature 
  digitalWrite(TEMP_PWR,HIGH);
  delay(50); 
  sensors.requestTemperatures(); // Send the command to get temperatures
  // We use the function ByIndex, and as an example get the temperature from the first (and only) sensor only.
  float tempC = sensors.getTempCByIndex(0);
  digitalWrite(TEMP_PWR,LOW);

  

  //Create the datastring for writing to SD card 
  datastring = datastring + "," + String(waterLevel) + "," + String(tempC);

  //Write datastring and close logfile on SD card
  dataFile = SD.open(filename[0], FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(datastring);
    dataFile.close();
  }

  //Check that the iridium modem is connected and the the clock has just reached midnight (i.e.,current time is within one logging interval of midnight)
  if (now.hour() == 0)
  {
    //Send daily data over Iridium
    send_daily_data(now);

  }

  //Write header if first time writing to the file
  if (!SD.exists("DAILY.CSV"))
  {
    //Write datastring and close logfile on SD card
    dataFile = SD.open("DAILY.CSV", FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("datetime,h2o_depth_mm,h2o_temp_deg_c");
      dataFile.close();
    }
  } else {
    //Write datastring and close logfile on SD card
    dataFile = SD.open("DAILY.CSV", FILE_WRITE);
    if (dataFile)
    {
      dataFile.println(datastring);
      dataFile.close();
    }
  }

}

void loop() {

  // We're done!
  // It's important that the donePin is written LOW and THEN HIGH. This shift
  // from low to HIGH is how the TPL5110 Nano Power Timer knows to turn off the
  // microcontroller.
  digitalWrite(DONEPIN, LOW);
  delay(10);
  digitalWrite(DONEPIN, HIGH);
  delay(10);
}
