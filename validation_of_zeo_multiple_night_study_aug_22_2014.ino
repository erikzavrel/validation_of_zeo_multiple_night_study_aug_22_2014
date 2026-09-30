// Program reads and logs raw data from Zeo Personal Sleep Coach

// Program will provide foundation for validation testing of Zeo as well as future research studies (REM sleep, depression, insomnia, long-term effects of alcohol on sleep architecture)

// Program is intended for use only with the Arduino Mega: program size is too large too run on Arduino Uno. Though no error will occur and program will compile (unlike with BS2) 
// to Arduino Uno board, program will behave erratically due to insufficient memory (2k vs. 8k SRAM and 32k vs. 256k flash). Need additional SRAM for large number of strings. 

// Erik Zavrel

// September 3 2012: most of program written summer 2012

// File names include month and date so files will not be overwritten if program is run multiple times without prior retrieval of files from SD card

// JUN 8 2013: May be desirable to log signal quality index and headband impedance measurement - to unique dedicated files?
// Reduced variable size allocation where possible (byte to boolean for program flags)

// JAN 26 2014: Solved reliability issue - pin assignment was wrong:

// chip_select_pin pin was set to 53 instead of 4
// CS pin is 4 for Uno and Mega
// SS pin (which must be set as output) is 10 on Uno and 53 on Mega

// Works 100% reliably now: no need for multiple resets of board to successfully intialize SD card (initalizes on first attempt every time and all files get successully created when headband
// is undocked and closed when headband is redocked)

// MAR 25 2014: Final inspection before commencing validation study, small changes, formatting

// MAR 27 2014: Added SQI and Headband Impedance files - may be used in review of data in comparison to PSG in validation study

// Can trust stage determination more when SQI is high and headband impedance is low (indicates good connection to forehead)

// APR 4 2014: Changed number_of_seconds_in_a_minute from byte to float data type. Essential, otherwise converting seconds awake, asleep, in a certain stage to minutes resulted in 
// erroneous values because integer not floating point math was being used: arithmetic operations are conducted using the data type of the operands -
// if operands are of different types, "larger" type is used for calculation

// APR 10 2014: Changed REM epoch duration counter and awakening duration counter. Previously, program had underestimated durations by 30 sec (0.5 min) because would only add 30 sec
// to duration if previous stage was not REM or awake. So if Zeo detected 1 stage of AWK, duration would appear as 0 not .5 min. If 3 consecutive stages of AWK, duration of awakening
// would appear as 1 min not 1 min 30 sec. Changed code so duration of awakening or rem epoch is always incremented by 30 sec and not only if previous stage was different.

// APR 11 2014: nomenclature / terminology changes:
// 
// total observation time changed to total recording time (TRT)
// total time asleep changed to total sleep time (TST)
// total time awake changed to total wake time (TWT)
// sleep onset event (as determined by Zeo) is actually onset of persistent sleep
// changed SOL (latency to sleep onset as judged by zeo) to latency to persistent sleep

// APR 15 2014:

// If first stage of sleep is REM, then REML will equal 0 min (zero latency).

// Needed to re-insert if sleep onset has occurred in calculation of awakening and waso durations, otherwise was including time to fall asleep as an awakening because first stages
// are always UNK

// APR 17 2014: Moved REM epoch counter and awakening counter to be incremented when epoch or duration is written to file. Otherwise, resulted in number of awakenings in summary file
// appearing as 1 more than the actual number (awk file) as previously number of awakenings was incremented when old sleep stage != awake and current sleep stage == awake. As awakenings
// were logged if / when they end, resulted in number of awakenings in summary file being 1 more than actual number (including final waking).

// AUG 22 2014: Corrected formatting (added space) between TST and (min) in header of output file

// Opening serial monitor (debug terminal) causes arduino board to be reset: as ribbon cable cannot be connected to serial port upon program startup, need to open serial monitor before 
// connecting ribbon cable (first open serial monitor - this will cause board to be reset - and then connect ribbon cable)

// The serial port is set at baud 38400, no parity, one stop bit, data is sent Least Significant Byte (LSB) first

// The serial protocol is:

//	AncllLLTttsid
//	A is a character starting the message
//	n is the protocol “version”, ie “4”
//	c is a one byte checksum formed by summing the identifier byte and all the data bytes
//	ll is a two byte message length sent LSB first. This length includes the size of the data block plus the identifier
//	LL is the inverse of ll sent for redundancy. If ll does not match ~LL, we can start looking for the start of the next block immediately, instead of reading some arbitrary number of bytes, based on a bad length
//	T is the lower 8 bits of Zeo’s unix time
//	tt is the 16-bit sub-second (runs through 0xFFFF in 1second), LSB first
//	s is an 8-bit sequence number
//	i is the datatype
//	d is the array of binary data

// ****************************************************************************************************************************************************

#include <SD.h> // include SD card library for logging data

// ****************************************************************************************************************************************************

// declaration of variables

const byte chip_select_pin = 4;
boolean SD_card_initialization_successful_flag = false; // initialize to false

File file_event_log; // log of events
File file_hypnogram; // sleep stage information for hypnogram
File file_EEG; // eeg data both absolute and relative
File file_night_summary; // summary of night with total minutes spent in each stage, sleep eff, number of awakenings etc
File file_REM; // rem epoch durations
File file_SQI; // SQI
File file_impedance; // headband impedance
File file_AWK; // awakening durations

const byte red_LED_pin = 8; // red status LED 
const byte yellow_LED_pin = 7; // yellow status LED
const byte green_LED_pin = 6; // green status LED

byte incoming_byte;
const char start_of_zeo_data_stream_header = 'A'; // "A" signifies start of data stream

const byte size_of_zeo_data_stream_header_array = 11;
byte zeo_data_stream_header_array[size_of_zeo_data_stream_header_array]; // 11 bytes in zeo data stream header, starting with protocol version and ending with data type
boolean received_zeo_data_header_complete_program_flag = false; // initialize to false

const byte size_of_zeo_data_stream_data_array = 14;
byte zeo_data_stream_data_array[size_of_zeo_data_stream_data_array]; // most data items to be captured from data stream is 14 (frequency bin data): fewer items for event type, signal quality index, time stamp, headband impedance, and sleep stage so this array size will accomodate them
boolean received_zeo_data_complete_program_flag = false; // initialize to false

byte data_stream_array_pointer; // for writing items from data stream to data array

byte number_of_bytes_in_data_block; // number of items to write to data array
byte zeo_data_type; // data type
byte event_type; // type of event
String event_message; // corresponding message of event to be used in log file
byte SQI; // quality of signal
byte impedance_across_headband_1; // impedance across headband byte 1 (impedance)
byte impedance_across_headband_2; // impedance across headband byte 2 (connection confirmation)
byte sleep_stage_type; // sleep stage
String sleep_stage_message;
byte sleep_stage_type_5s_complement_for_hypnogram_reconstruction; // 5's complement of zeo's sleep stage numbering scheme to reconstruct hypnogram with appropriate convention

// data type dictionary:
const byte data_type_event = 0; // 0x00 : An event has occurred
const byte data_type_frequency_bins = 131; // 0x83 : Frequency bins derived from waveform
const byte data_type_signal_quality_index = 132; // 0x84 : Signal Quality Index of waveform (0-30), 0 when sensor is not worn, 30 when sensor is in good contact, in-between when user is furrowing brow while wearing sensor
const byte data_type_time_stamp = 138; // 0x8A : Timestamp from Zeo’s RTC
const byte data_type_impedance = 151; // 0x97 : Impedance across headband
const byte data_type_sleep_stage = 157; // 0x9D : Current 30 sec sleep stage

// event dictionary:
const byte event_night_start = 5; // 0x05 : ‘NightStart’ : User’s night has begun
const byte event_sleep_onset = 7; // 0x07 : ‘SleepOnset' : User is asleep
const byte event_headband_docked = 14; // 0x0E : ‘HeadbandDocked’ : Headband returned to dock
const byte event_headband_undocked = 15; // 0x0F : ‘HeadbandUnDocked’ : Headband removed from dock
const byte event_alarm_off = 16; // 0x10 : ‘AlarmOff’ : User turned off the alarm
const byte event_alarm_snooze = 17; // 0x11 : ‘AlarmSnooze’ : User hit snooze
const byte event_alarm_play = 19; // 0x13 : ‘AlarmPlay’ : Alarm is firing
const byte event_night_end = 21; // 0x15 : ‘NightEnd’ : User’s night has ended
const byte event_new_headband = 36; // 0x24 : ‘NewHeadband’ : A new headband ID has been read

String event_message_night_start = "Night Start";
String event_message_sleep_onset = "Sleep Onset";
String event_message_headband_docked = "Headband Docked";
String event_message_headband_undocked = "Headband Undocked";
String event_message_alarm_off = "Alarm Turned Off";
String event_message_alarm_snooze = "Alarm Snooze Hit";
String event_message_alarm_play = "Alarm Playing";
String event_message_night_end = "Night Has Ended";
String event_message_new_headband = "New Headband";

// frequency bin dictionary:
unsigned int brain_wave_delta; // 2-4 Hz
unsigned int brain_wave_theta; // 4-8 Hz
unsigned int brain_wave_alpha; // 8-13 Hz
unsigned int brain_wave_beta_13_to_18; // 13-18 Hz
unsigned int brain_wave_beta_18_to_21; // 18-21 Hz
unsigned int brain_wave_beta_11_to_14_sleep_spindles; // 11-14 Hz
unsigned int brain_wave_gamma; // 30-50 Hz

float total_brain_wave_activity; // arithmetic operations are conducted using the data type of the operands: if operands are of different types, "larger" type is used for calculation

float brain_wave_delta_percentage; // float variable type needed for decimal values
float brain_wave_theta_percentage; // float variable type needed for decimal values
float brain_wave_alpha_percentage; // float variable type needed for decimal values
float brain_wave_beta_13_to_18_percentage; // float variable type needed for decimal values
float brain_wave_beta_18_to_21_percentage; // float variable type needed for decimal values
float brain_wave_beta_11_to_14_sleep_spindles_percentage; // float variable type needed for decimal values
float brain_wave_gamma_percentage; // float variable type needed for decimal values

// time stamp dictionary
byte unix_time_stamp_least_significant_byte; // LSB of unix time stamp
byte unix_time_stamp_second_least_significant_byte; // 2nd LSB of unix time stamp
byte unix_time_stamp_second_most_significant_byte; // 2nd MSB of unix time stamp
byte unix_time_stamp_most_significant_byte; // MSB of unix time stamp

unsigned long unix_time_stamp_least_significant_byte_weighed; // LSB of unix time stamp weighed
unsigned long unix_time_stamp_second_least_significant_byte_weighed; // 2nd LSB of unix time stamp weighed
unsigned long unix_time_stamp_second_most_significant_byte_weighed; // 2nd MSB of unix time stamp weighed
unsigned long unix_time_stamp_most_significant_byte_weighed; // MSB of unix time stamp weighed

unsigned long unix_time_seconds_since_unix_epoch; // seconds elapsed since Jan 1 1970

unsigned long number_of_complete_years_since_1970; // variable type must be large enough to hold intermediate calculations
const unsigned long number_of_seconds_in_a_day = 86400; // 60 sec / min, 60 min / hr, 24 hrs / day
unsigned long current_year; // 1970 + number of elapsed years
int year; // for counting number of leap years since unix epoch
unsigned long number_of_leap_years_since_unix_epoch; // must take into account leap years have 366 days not 365
unsigned long number_of_seconds_in_leap_days; // number of seconds in those extra days from leap years since 1970
unsigned long number_of_seconds_of_current_year;
unsigned long number_of_complete_days_in_current_year;
unsigned long current_day_of_current_year; // current day is incomplete
byte current_month; // current month
unsigned long current_date; // current date
unsigned long number_of_seconds_in_current_day;
unsigned long current_hour; // current hour
unsigned long number_of_seconds_in_current_hour;
unsigned long current_minute; // current minute
unsigned long current_second; // current second

boolean connection_established_between_zeo_and_arduino_program_flag = false; // set upon receiving first time stamp from Zeo RTC - initialize to false

byte current_month_tens_place; // 1st character for file names: MMDD_XXX.csv
byte current_month_ones_place; // 2nd character for file names: MMDD_XXX.csv
byte current_date_tens_place; // 3rd character for file names: MMDD_XXX.csv
byte current_date_ones_place; // 4th character for file names: MMDD_XXX.csv

const byte ascii_offset = 48; // to convert to ascii character representation

byte current_month_tens_place_ascii_representation; // 1st character for file names: MMDD_XXX.csv
byte current_month_ones_place_ascii_representation; // 2nd character for file names: MMDD_XXX.csv
byte current_date_tens_place_ascii_representation; // 3rd character for file names: MMDD_XXX.csv
byte current_date_ones_place_ascii_representation; // 4th character for file names: MMDD_XXX.csv

char current_month_tens_place_for_file_name; // 1st character for file names: MMDD_XXX.csv
char current_month_ones_place_for_file_name; // 2nd character for file names: MMDD_XXX.csv
char current_date_tens_place_for_file_name; // 3rd character for file names: MMDD_XXX.csv
char current_date_ones_place_for_file_name; // 4th character for file names: MMDD_XXX.csv

const byte current_month_tens_place_file_name_element_number = 0; // will overwrite 0th element in array = 1st character for file names: MMDD_XXX.csv
const byte current_month_ones_place_file_name_element_number = 1; // will overwrite 1st element in array = 2nd character for file names: MMDD_XXX.csv
const byte current_date_tens_place_file_name_element_number = 2; // will overwrite 2nd element in array = 3rd character for file names: MMDD_XXX.csv
const byte current_date_ones_place_file_name_element_number = 3; // will overwrite 3rd element in array = 4th character for file names: MMDD_XXX.csv

char file_name_event_log[] = "MMDD_LOG.csv"; // log of events
char file_name_hypnogram[] = "MMDD_HYP.csv"; // sleep stage information for hypnogram
char file_name_EEG[] = "MMDD_EEG.csv"; // eeg data both absolute and relative
char file_name_summary[] = "MMDD_SUM.csv"; // summary of night with total minutes spent in each stage, sleep eff, number of awakenings etc
char file_name_REM[] = "MMDD_REM.csv"; // rem epochs
char file_name_SQI[] = "MMDD_SQI.csv"; // signal quality index
char file_name_impedance[] = "MMDD_IMP.csv"; // headband impedance
char file_name_AWK[] = "MMDD_AWK.csv"; // awakenings

// sleep stage dictionary:
const byte sleep_stage_undefined = 0; // Sleep stage unsure
const byte sleep_stage_awake = 1; // Awake
const byte sleep_stage_rem = 2; // Rapid eye movement(possibly dreaming)
const byte sleep_stage_light = 3; // Light sleep
const byte sleep_stage_deep = 4; // Deep sleep

String sleep_stage_message_undefined = "UNK";
String sleep_stage_message_awake = "AWK";
String sleep_stage_message_rem = "REM";
String sleep_stage_message_light = "L";
String sleep_stage_message_deep = "D";

byte sleep_stage_type_old; // last sleep stage for comparison to determine whether a new awakening or rem epoch has occurred or is just a continuation of an already counted awakening or rem epoch

unsigned long unix_time_seconds_since_unix_epoch_headband_removed_from_dock; // time when observation begins
unsigned long unix_time_seconds_since_unix_epoch_headband_returned_to_dock; // time when observation ends
unsigned long total_recording_time_seconds = 0; // elapsed time from headband removal from dock to headband return to dock = length of night (unsigned int variable size can store only up to 18 hours - may not be sufficient)
const float number_of_seconds_in_a_minute = 60.0; // for converting values in seconds to values in minutes: arithmetic operations are conducted using the data type of the operands - if operands are of different types, "larger" type is used for calculation
float total_recording_time_minutes = 0; // elapsed time from headband removal from dock to headband return to dock = length of night

unsigned long total_seconds_unknown_sleep_stage = 0; // total seconds in unknown sleep stage
unsigned long total_seconds_awake_sleep_stage = 0; // total seconds in awake stage
unsigned long total_seconds_rem_sleep_stage = 0; // total seconds in rem sleep stage
unsigned long total_seconds_light_sleep_stage = 0; // total seconds in light sleep stage
unsigned long total_seconds_deep_sleep_stage = 0; // total seconds in deep sleep stage

float total_minutes_unknown_sleep_stage = 0; // total minutes in unknown sleep stage
float total_minutes_awake_sleep_stage = 0; // total minutes in awake stage
float total_minutes_rem_sleep_stage = 0; // total minutes in rem sleep stage
float total_minutes_light_sleep_stage = 0; // total minutes in light sleep stage
float total_minutes_deep_sleep_stage = 0; // total minutes in deep sleep stage

unsigned long total_sleep_time_seconds = 0; // seconds spent in either L, D, or REM stage (definite sleep)
float total_sleep_time_minutes = 0; // minutes spent in either L, D, or REM stage (definite sleep)

float rem_sleep_stage_percentage; // percentage of sleep that is REM
float light_sleep_stage_percentage; // percentage of sleep that is L
float deep_sleep_stage_percentage; // percentage of sleep that is D

float sleep_efficiency_percentage; // total_sleep_time_seconds / total_recording_time_seconds * 100%

unsigned long unix_time_seconds_since_unix_epoch_sleep_onset; // time of first epoch of sleep
unsigned long sleep_onset_latency_seconds = 0; // number of seconds from headband undocking to first epoch of sleep
float sleep_onset_latency_minutes = 0; // number of minutes from headband undocking to first epoch of sleep
boolean sleep_onset_program_flag = false; // whether sleep onset has occurred or not - initialize to false

unsigned long unix_time_seconds_since_unix_epoch_persistent_sleep; // time when sleep onset occurs as determined by Zeo
unsigned long latency_to_persistent_sleep_seconds = 0; // number of seconds from headband undocking to sleep onset
float latency_to_persistent_sleep_minutes = 0; // number of minutes from headband undocking to sleep onset
boolean persistent_sleep_program_flag = false; // whether sleep onset has occurred or not - initialize to false

unsigned int number_of_awakenings = 0; // number of instances of being awakened throughout course of night
unsigned int awakening_duration_seconds = 0;
float awakening_duration_minutes = 0;

float wakefulness_after_sleep_onset_minutes = 0;

unsigned long unix_time_seconds_since_unix_epoch_rem_onset; // time when rem onset occurs
unsigned long rem_onset_latency_seconds = 0; // number of seconds from sleep onset to first rem episode
float rem_onset_latency_minutes = 0; // number of minutes from sleep onset to first rem episode
boolean rem_onset_program_flag = false; // whether rem sleep has occurred or not - initialize to false

unsigned int number_of_rem_epochs = 0; // number of instances / blocks of rem sleep
unsigned int rem_epoch_duration_seconds = 0;
float rem_epoch_duration_minutes = 0;

// ****************************************************************************************************************************************************

void setup() {
  
  delay(1000); // pause 1 sec upon startup / allow for settling
 
  Serial.begin(38400); // opens serial port to the zeo, sets data rate to 38400 bps
  
  pinMode(red_LED_pin, OUTPUT); // configure LED for output
  pinMode(yellow_LED_pin, OUTPUT); // configure LED for output
  pinMode(green_LED_pin, OUTPUT); // configure LED for output
  
  pinMode(53, OUTPUT); // hardware SS pin must be left as an output or the SD library won't work, 10 ON ARDUINO UNO, 53 ON ARDUINO MEGA
  
  digitalWrite(red_LED_pin, HIGH); // RED indicator LED ON
  digitalWrite(yellow_LED_pin, HIGH); // YELLOW indicator LED ON
  digitalWrite(green_LED_pin, HIGH); // GREEN indicator LED ON
  
  delay(1000); // pause 1 second
  
  // at start of program, turn on all indicator LEDs
  
  // Must initialize SD card and create and open files. Don't connect ribbon cable from Zeo to Arduino RX pin yet.
  // If Zeo is connected to Arduino when power is connected or when board is reset, program will hang / stall!
  // Create and open files upon removal of headband from dock. This represents start of night so if this event is somehow missed, entire study is compromised.
  // Also will need to get date and time from Zeo RTC for multiple night studies as files will need to be named using the date to prevent file overwrites!
   
  while (SD_card_initialization_successful_flag == false) { // if card has not been initialized, try to initialize it
    
    if (SD.begin(chip_select_pin)) {
      SD_card_initialization_successful_flag = true; // if initialization sucessful, set the program flag
    } // end if
    
  } // end while
  
  digitalWrite(red_LED_pin, LOW); // RED indicator LED OFF
  digitalWrite(yellow_LED_pin, LOW); // YELLOW indicator LED OFF
  digitalWrite(green_LED_pin, LOW); // GREEN indicator LED OFF
  
  // turn off all indicator LEDS to indicate card has been initialized successfully and user may connect ribbon cable
 
} // end setup

// ****************************************************************************************************************************************************

void loop() { // wait for data

  if (Serial.available() > 0) { // if there is a byte to read
    
    incoming_byte = Serial.read(); // read the incoming byte
    
    if (incoming_byte == start_of_zeo_data_stream_header) { // "A" signifies start of Zeo data stream
      read_zeo_data_stream_header(); // new data is starting
    } // end if
    
  } // end if
  
} // loop until start of data stream is indicated by header character

// ****************************************************************************************************************************************************

void read_zeo_data_stream_header() { // read header portion of data stream

  data_stream_array_pointer = 0; // reset pointer to 0

  while (received_zeo_data_header_complete_program_flag == false) {
    
    if (Serial.available() > 0) { // if there is a byte to read
      
      zeo_data_stream_header_array[data_stream_array_pointer] = Serial.read(); // read the incoming byte to data stream header array
      data_stream_array_pointer = data_stream_array_pointer + 1; // increment array pointer
      
      if (data_stream_array_pointer == size_of_zeo_data_stream_header_array) { // 11 bytes in zeo data stream header (0-10), starting with protocol version and ending with data type
        received_zeo_data_header_complete_program_flag = true; // set program flag
      } // end if
      
    } // end if
    
  } // end while
        
  received_zeo_data_header_complete_program_flag = false; // reset program flag

  if ((zeo_data_stream_header_array[2] + zeo_data_stream_header_array[4] == 255) && (zeo_data_stream_header_array[3] + zeo_data_stream_header_array[5] == 255)) { // if ll does not match LL, can start looking for the start of the next block immediately, instead of reading some arbitrary number of bytes, based on a bad length
    
    number_of_bytes_in_data_block = zeo_data_stream_header_array[2] + zeo_data_stream_header_array[3] - 1; // third and fourth elements in header array are size of the data block plus the identifier (already included in header array so number of data items minus identifier = size of data array = number of items it contains                                           
    
    zeo_data_type = zeo_data_stream_header_array[10]; // last element in header array is the data type

    if ((zeo_data_type == data_type_event) || (zeo_data_type == data_type_frequency_bins) || (zeo_data_type == data_type_signal_quality_index) || (zeo_data_type == data_type_time_stamp) || (zeo_data_type == data_type_impedance) || (zeo_data_type == data_type_sleep_stage)) { // only if the data typer is one of these - not interested in anything else
      
      read_zeo_data_stream_data(); // read data elements
   
    } // end if
    
  } // end if

  return;
  
} // end of read_zeo_data_stream_header() function

// ****************************************************************************************************************************************************

void read_zeo_data_stream_data() {

  data_stream_array_pointer = 0; // reset array pointer to 0 for writing elements to data array
  
  while(received_zeo_data_complete_program_flag == false){
    
    if (Serial.available() > 0) { // if there is a byte to read
    
      zeo_data_stream_data_array[data_stream_array_pointer] = Serial.read(); // read the incoming byte to data stream data array
      data_stream_array_pointer = data_stream_array_pointer + 1; // increment array pointer
      
      if (data_stream_array_pointer == number_of_bytes_in_data_block) {
        received_zeo_data_complete_program_flag = true; // set program flag
      } // end if
      
    } // end if
    
  } // end while
    
  received_zeo_data_complete_program_flag = false; // reset program flag

  switch (zeo_data_type) { // choose action based on data type
    case data_type_event:
      event(); // handles events
      break; // break out of selection process otherwise command under next case will execute
    case data_type_frequency_bins:
      eeg_data(); // handles eeg data
      break; // break out of selection process otherwise command under next case will execute
    case data_type_signal_quality_index:
      signal_quality_index(); // handles sqi
      break; // break out of selection process otherwise command under next case will execute
    case data_type_time_stamp:
      convert_from_unix_time(); // converts zeo unix time to format suitable for log files
      break; // break out of selection process otherwise command under next case will execute
    case data_type_impedance:
      headband_impedance(); // handles headband impedance
      break; // break out of selection process otherwise command under next case will execute
    case data_type_sleep_stage:
      sleep_stage(); // handles sleep stage
      break; // break out of selection process otherwise command under next case will execute
  } // end switch
  
  return;
  
} // end of read_zeo_data_stream_data() function

// ****************************************************************************************************************************************************

void event() {
  
  event_type = zeo_data_stream_data_array[0]; // when data type is event, first item (element 0) in data stream is the type of event
  
  switch (event_type) { // determine appropriate message for event
    case event_night_start:
      event_message = event_message_night_start;
      break; // break out of selection process otherwise command under next case will execute
    case event_sleep_onset:
      event_message = event_message_sleep_onset; 
      if (persistent_sleep_program_flag == false) { // if this is the first time sleep onset event has occurred
        unix_time_seconds_since_unix_epoch_persistent_sleep = unix_time_seconds_since_unix_epoch; // time of sleep onset set to most recent time stamp from RTC
        persistent_sleep_program_flag = true; // set the flag so that this code block does not execute again
      } // end if
      break; // break out of selection process otherwise command under next case will execute
    case event_headband_docked:
      event_message = event_message_headband_docked; 
      unix_time_seconds_since_unix_epoch_headband_returned_to_dock = unix_time_seconds_since_unix_epoch; // time of observation end set to most recent time stamp from RTC
      break; // break out of selection process otherwise command under next case will execute
    case event_headband_undocked:
      event_message = event_message_headband_undocked;
      unix_time_seconds_since_unix_epoch_headband_removed_from_dock = unix_time_seconds_since_unix_epoch; // time of observation start set to most recent time stamp from RTC
      create_and_open_files(); // function that creates and opens files
      break; // break out of selection process otherwise command under next case will execute
    case event_alarm_off:
      event_message = event_message_alarm_off; 
      break; // break out of selection process otherwise command under next case will execute
    case event_alarm_snooze:
      event_message = event_message_alarm_snooze; 
      break; // break out of selection process otherwise command under next case will execute
    case event_alarm_play:
      event_message = event_message_alarm_play; 
      break; // break out of selection process otherwise command under next case will execute
    case event_night_end:
      event_message = event_message_night_end; 
      break; // break out of selection process otherwise command under next case will execute
    case event_new_headband:
      event_message = event_message_new_headband; 
      break; // break out of selection process otherwise command under next case will execute     
  } // end switch
 
  if (file_event_log) { // if the file opened okay, write to it:
  
    file_event_log.print(current_year);
    file_event_log.print(","); // comma delimited fields
    file_event_log.print(current_month);
    file_event_log.print(","); // comma delimited fields
    file_event_log.print(current_date);
    file_event_log.print(","); // comma delimited fields
    file_event_log.print(current_hour);
    file_event_log.print(","); // comma delimited fields
    file_event_log.print(current_minute);
    file_event_log.print(","); // comma delimited fields
    file_event_log.print(current_second);
    file_event_log.print(","); // comma delimited fields
    file_event_log.println(event_message); // last item written to file must be printed using "println" to move to next line
    
  } // end if
  
  if (event_type == event_headband_docked) { // headband may be returned to dock before night start, would prefer program to end after night end as night end comes after headband docked (assuming night start has already occurred)
    
    total_recording_time_seconds = unix_time_seconds_since_unix_epoch_headband_returned_to_dock - unix_time_seconds_since_unix_epoch_headband_removed_from_dock; // total observation time
    sleep_onset_latency_seconds = unix_time_seconds_since_unix_epoch_sleep_onset - unix_time_seconds_since_unix_epoch_headband_removed_from_dock; // seconds from headband undocking to sleep onset (first epoch of REM, L, or D)
    latency_to_persistent_sleep_seconds = unix_time_seconds_since_unix_epoch_persistent_sleep - unix_time_seconds_since_unix_epoch_headband_removed_from_dock; // seconds from headband undocking to persistent sleep (sleep onset as determined by Zeo)
    rem_onset_latency_seconds = unix_time_seconds_since_unix_epoch_rem_onset - unix_time_seconds_since_unix_epoch_sleep_onset; // seconds from sleep onset to rem onset
    
    total_recording_time_minutes = total_recording_time_seconds / number_of_seconds_in_a_minute; // convert from seconds to minutes
    sleep_onset_latency_minutes = sleep_onset_latency_seconds / number_of_seconds_in_a_minute; // convert from seconds to minutes
    latency_to_persistent_sleep_minutes = latency_to_persistent_sleep_seconds / number_of_seconds_in_a_minute; // convert from seconds to minutes
    rem_onset_latency_minutes = rem_onset_latency_seconds / number_of_seconds_in_a_minute; // convert from seconds to minutes
    
    total_minutes_unknown_sleep_stage = total_seconds_unknown_sleep_stage / number_of_seconds_in_a_minute; // convert from seconds to minutes
    total_minutes_awake_sleep_stage = total_seconds_awake_sleep_stage / number_of_seconds_in_a_minute; // convert from seconds to minutes
    total_minutes_rem_sleep_stage = total_seconds_rem_sleep_stage / number_of_seconds_in_a_minute; // convert from seconds to minutes
    total_minutes_light_sleep_stage = total_seconds_light_sleep_stage / number_of_seconds_in_a_minute; // convert from seconds to minutes
    total_minutes_deep_sleep_stage = total_seconds_deep_sleep_stage / number_of_seconds_in_a_minute; // convert from seconds to minutes
    
    total_sleep_time_minutes = total_sleep_time_seconds / number_of_seconds_in_a_minute; // convert from seconds to minutes
    
    rem_sleep_stage_percentage = (total_minutes_rem_sleep_stage / total_sleep_time_minutes) * 100; // percentage of sleep that is REM
    light_sleep_stage_percentage = (total_minutes_light_sleep_stage / total_sleep_time_minutes) * 100; // percentage of sleep that is L
    deep_sleep_stage_percentage = (total_minutes_deep_sleep_stage / total_sleep_time_minutes) * 100; // percentage of sleep that is D    
   
    sleep_efficiency_percentage = (total_sleep_time_minutes / total_recording_time_minutes) * 100;
    
    if (file_night_summary) { // if the file opened okay, write to it:
    
      file_night_summary.print(total_recording_time_minutes);
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(total_sleep_time_minutes);
      file_night_summary.print(","); // comma delimited fields  
      file_night_summary.print(total_minutes_awake_sleep_stage);
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(sleep_efficiency_percentage);
      file_night_summary.print(","); // comma delimited fields
      
      if ((total_minutes_unknown_sleep_stage > total_recording_time_minutes) || (total_minutes_unknown_sleep_stage == 0)) {
        total_minutes_unknown_sleep_stage = total_recording_time_minutes;
      } // end if
      
      // safeguard to ensure if number of seconds of unknown stage doesn't exceed total duration of study
      // first determination of sleep stage occurs less than 30 seconds after headband is removed from dock and is always unknown, 
      // necessary to keep total number of seconds of unknown stage from being greater than total seconds of observation
      
      file_night_summary.print(total_minutes_unknown_sleep_stage);
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(total_minutes_light_sleep_stage);
      file_night_summary.print(","); // comma delimited fields  
      file_night_summary.print(total_minutes_deep_sleep_stage);
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(total_minutes_rem_sleep_stage);
      file_night_summary.print(","); // comma delimited fields
      
      file_night_summary.print(light_sleep_stage_percentage);
      file_night_summary.print(","); // comma delimited fields  
      file_night_summary.print(deep_sleep_stage_percentage);
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(rem_sleep_stage_percentage);
      file_night_summary.print(","); // comma delimited fields      
      
      if (sleep_onset_program_flag == true) {
        file_night_summary.print(sleep_onset_latency_minutes);
      } // end if
      
      else {
        file_night_summary.print("N/A"); // number of seconds to sleep onset only exists if person fell asleep
      } // end else

      file_night_summary.print(","); // comma delimited fields
      
      if (persistent_sleep_program_flag == true) {
        file_night_summary.print(latency_to_persistent_sleep_minutes);
      } // end if
      
      else {
        file_night_summary.print("N/A"); // number of seconds to sleep onset only exists if person fell asleep
      } // end else

      file_night_summary.print(","); // comma delimited fields

      if (sleep_onset_program_flag == true) {
        file_night_summary.print(wakefulness_after_sleep_onset_minutes);
      } // end if
      
      else {
        file_night_summary.print("N/A"); // number of seconds to sleep onset only exists if person fell asleep
      } // end else
      
      file_night_summary.print(","); // comma delimited fields  
      file_night_summary.print(number_of_awakenings);
      file_night_summary.print(","); // comma delimited fields
      
      if (rem_onset_program_flag == true) {
        file_night_summary.print(rem_onset_latency_minutes);
      } // end if
      
      else {
        file_night_summary.print("N/A"); // number of seconds to rem onset only exists if person had at least one rem episode
      } // end else
    
      file_night_summary.print(","); // comma delimited fields
      file_night_summary.print(number_of_rem_epochs);
   
    } // end if

    do { // execute the code block at least one time:
      file_event_log.close(); // close the file
      file_hypnogram.close(); // close the file
      file_EEG.close(); // close the file
      file_night_summary.close(); // close the file
      file_REM.close(); // close the file
      file_SQI.close(); // close the file
      file_impedance.close(); // close the file
      file_AWK.close(); // close the file
    } while ((file_event_log) || (file_hypnogram) || (file_EEG) || (file_night_summary) || (file_REM) || (file_SQI) || (file_impedance) || (file_AWK)); // if files didn't close, keep trying to close them (will execute close commands if any of the files are still open)
    
    // if ANY of the files failed to close properly, keep looping attempting to close them until they are all closed, close command on already closed file has no effect
    
    digitalWrite(red_LED_pin, LOW); // RED indicator LED OFF
    digitalWrite(yellow_LED_pin, LOW); // YELLOW indicator LED OFF
    digitalWrite(green_LED_pin, HIGH); // GREEN indicator LED ON
      
    // files closed: can safely eject SD card
    
    while (1 == 1) { // always true
      delay(1000);
    } // infinite loop to keep program here after closing of files otherwise program may try to write data to closed files if headband is undocked again
  
  } // end if
  
  return;
  
} // end of event() function

// ****************************************************************************************************************************************************

void eeg_data() {
  
  brain_wave_delta = zeo_data_stream_data_array[0] + (256 * zeo_data_stream_data_array[1]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first 
  brain_wave_theta = zeo_data_stream_data_array[2] + (256 * zeo_data_stream_data_array[3]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first  
  brain_wave_alpha = zeo_data_stream_data_array[4] + (256 * zeo_data_stream_data_array[5]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first 
  brain_wave_beta_13_to_18 = zeo_data_stream_data_array[6] + (256 * zeo_data_stream_data_array[7]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first  
  brain_wave_beta_18_to_21 = zeo_data_stream_data_array[8] + (256 * zeo_data_stream_data_array[9]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first  
  brain_wave_beta_11_to_14_sleep_spindles = zeo_data_stream_data_array[10] + (256 * zeo_data_stream_data_array[11]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first  
  brain_wave_gamma = zeo_data_stream_data_array[12] + (256 * zeo_data_stream_data_array[13]); // absolute intensity, each eeg value 2 bytes long with data sent LSB first  
  
  total_brain_wave_activity = brain_wave_delta + brain_wave_theta + brain_wave_alpha + brain_wave_beta_13_to_18 + brain_wave_beta_18_to_21 + brain_wave_beta_11_to_14_sleep_spindles + brain_wave_gamma; // sum of all activity 
  
  brain_wave_delta_percentage = (brain_wave_delta / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_theta_percentage = (brain_wave_theta / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_alpha_percentage = (brain_wave_alpha / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_beta_13_to_18_percentage = (brain_wave_beta_13_to_18 / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_beta_18_to_21_percentage = (brain_wave_beta_18_to_21 / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_beta_11_to_14_sleep_spindles_percentage = (brain_wave_beta_11_to_14_sleep_spindles / total_brain_wave_activity) * 100; // relative intensity 
  brain_wave_gamma_percentage = (brain_wave_gamma / total_brain_wave_activity) * 100; // relative intensity 
  
  if (file_EEG) { // if the file is available, write to it:
  
    file_EEG.print(current_year);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(current_month);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(current_date);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(current_hour);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(current_minute);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(current_second);
    file_EEG.print(","); // comma delimited fields
    
    file_EEG.print(brain_wave_delta);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_theta);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_alpha);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_13_to_18);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_18_to_21);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_11_to_14_sleep_spindles);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_gamma);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(total_brain_wave_activity);
    file_EEG.print(","); // comma delimited fields
    
    file_EEG.print(brain_wave_delta_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_theta_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_alpha_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_13_to_18_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_18_to_21_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.print(brain_wave_beta_11_to_14_sleep_spindles_percentage);
    file_EEG.print(","); // comma delimited fields
    file_EEG.println(brain_wave_gamma_percentage); // last item written to file must be printed using "println" to move to next line

  } // end if
  
  return;
  
} // end of void eeg_data() function

// ****************************************************************************************************************************************************

void signal_quality_index() {
  
  SQI = zeo_data_stream_data_array[0]; // when data type is SQI, first item (element 0) in data stream is the SQI value
  
  if (file_SQI) { // if the file is available, write to it:
  
    file_SQI.print(current_year);
    file_SQI.print(","); // comma delimited fields
    file_SQI.print(current_month);
    file_SQI.print(","); // comma delimited fields
    file_SQI.print(current_date);
    file_SQI.print(","); // comma delimited fields
    file_SQI.print(current_hour);
    file_SQI.print(","); // comma delimited fields
    file_SQI.print(current_minute);
    file_SQI.print(","); // comma delimited fields
    file_SQI.print(current_second);
    file_SQI.print(","); // comma delimited fields
    
    file_SQI.println(SQI); // last item written to file must be printed using "println" to move to next line
    
    // Note: first two SQI measurements in SQI file have same time stamp (down to same second)
    // Explanation: SQI is put out each second. Upon connection of ribbon cable, arduino obtains first time stamp 
    // Code overhead with receiving first time stamp results in missing next second from Zeo, resulting in first two SQI values having same (current) time stamp
    
  } // end if
    
  return;
  
} // end of signal_quality_index() function

// ****************************************************************************************************************************************************

void convert_from_unix_time() {
  
  unix_time_stamp_least_significant_byte = zeo_data_stream_data_array[0];
  unix_time_stamp_second_least_significant_byte = zeo_data_stream_data_array[1];
  unix_time_stamp_second_most_significant_byte = zeo_data_stream_data_array[2];
  unix_time_stamp_most_significant_byte = zeo_data_stream_data_array[3];
  
  unix_time_stamp_least_significant_byte_weighed = (unix_time_stamp_least_significant_byte * pow(256,0)); // TIMES 1
  unix_time_stamp_second_least_significant_byte_weighed = (unix_time_stamp_second_least_significant_byte * pow(256,1)); // TIMES 256^1
  unix_time_stamp_second_most_significant_byte_weighed = (unix_time_stamp_second_most_significant_byte * pow(256,2)); // TIMES 256^2
  unix_time_stamp_most_significant_byte_weighed = (unix_time_stamp_most_significant_byte * pow(256,3)); // TIMES 256^3
  
  unix_time_seconds_since_unix_epoch = unix_time_stamp_most_significant_byte_weighed + unix_time_stamp_second_most_significant_byte_weighed + unix_time_stamp_second_least_significant_byte_weighed + unix_time_stamp_least_significant_byte_weighed; // add weighed components of unix time to arrive at total number of seconds elapsed since jan 1 1970
  
  number_of_complete_years_since_1970 = unix_time_seconds_since_unix_epoch / (number_of_seconds_in_a_day * 365);
  current_year = number_of_complete_years_since_1970 + 1970; // unix time starts Jan 1 1970
  
  number_of_leap_years_since_unix_epoch = 0; // must reset to 0
  
  for (year = 1970; year < current_year; year = year + 1) {
    if (year % 4 == 0) { // if remainder = 0 when divided by 4
      number_of_leap_years_since_unix_epoch = number_of_leap_years_since_unix_epoch + 1; // add to running total
    } // end if
  } // find number of leap years since unix epoch
      
  number_of_seconds_in_leap_days = number_of_seconds_in_a_day * number_of_leap_years_since_unix_epoch;
  number_of_seconds_of_current_year = (unix_time_seconds_since_unix_epoch % (number_of_seconds_in_a_day * 365)) - number_of_seconds_in_leap_days;
  number_of_complete_days_in_current_year = number_of_seconds_of_current_year / number_of_seconds_in_a_day;
  current_day_of_current_year = number_of_complete_days_in_current_year + 1; // current day is incomplete 
  
  if (current_year % 4 != 0) { // if this year is NOT a leap year then FEB has 28 days
    if (current_day_of_current_year <= 31) {
      current_month = 1;
      current_date = current_day_of_current_year;
    }
    else if ((current_day_of_current_year > 31) && (current_day_of_current_year <= 59)) {
      current_month = 2;
      current_date = current_day_of_current_year - 31;
    }
    else if ((current_day_of_current_year > 59) && (current_day_of_current_year <= 90)) {
      current_month = 3;
      current_date = current_day_of_current_year - 59;
    }
    else if ((current_day_of_current_year > 90) && (current_day_of_current_year <= 120)) {
      current_month = 4;
      current_date = current_day_of_current_year - 90;
    }
    else if ((current_day_of_current_year > 120) && (current_day_of_current_year <= 151)) {
      current_month = 5;
      current_date = current_day_of_current_year - 120;
    }
    else if ((current_day_of_current_year > 151) && (current_day_of_current_year <= 181)) {
      current_month = 6;
      current_date = current_day_of_current_year - 151;
    }
    else if ((current_day_of_current_year > 181) && (current_day_of_current_year <= 212)) {
      current_month = 7;
      current_date = current_day_of_current_year - 181;
    }
    else if ((current_day_of_current_year > 212) && (current_day_of_current_year <= 243)) {
      current_month = 8;
      current_date = current_day_of_current_year - 212;
    }
    else if ((current_day_of_current_year > 243) && (current_day_of_current_year <= 273)) {
      current_month = 9;
      current_date = current_day_of_current_year - 243;
    }
    else if ((current_day_of_current_year > 273) && (current_day_of_current_year <= 304)) {
      current_month = 10;
      current_date = current_day_of_current_year - 273;
    }
    else if ((current_day_of_current_year > 304) && (current_day_of_current_year <= 334)) {
      current_month = 11;
      current_date = current_day_of_current_year - 304;
    }
    else if (current_day_of_current_year > 334) {
      current_month = 12;
      current_date = current_day_of_current_year - 334;
    }
  } // END IF YEAR IS NOT A LEAP YEAR
  
  else if (current_year % 4 == 0) { // if this year IS a leap year then FEB has 29 days
    if (current_day_of_current_year <= 31) {
      current_month = 1;
      current_date = current_day_of_current_year;
    }
    else if ((current_day_of_current_year > 31) && (current_day_of_current_year <= 60)) {
      current_month = 2;
      current_date = current_day_of_current_year - 31;
    }
    else if ((current_day_of_current_year > 60) && (current_day_of_current_year <= 91)) {
      current_month = 3;
      current_date = current_day_of_current_year - 60;
    }
    else if ((current_day_of_current_year > 91) && (current_day_of_current_year <= 121)) {
      current_month = 4;
      current_date = current_day_of_current_year - 91;
    }
    else if ((current_day_of_current_year > 121) && (current_day_of_current_year <= 152)) {
      current_month = 5;
      current_date = current_day_of_current_year - 121;
    }
    else if ((current_day_of_current_year > 152) && (current_day_of_current_year <= 182)) {
      current_month = 6;
      current_date = current_day_of_current_year - 152;
    }
    else if ((current_day_of_current_year > 182) && (current_day_of_current_year <= 213)) {
      current_month = 7;
      current_date = current_day_of_current_year - 182;
    }
    else if ((current_day_of_current_year > 213) && (current_day_of_current_year <= 244)) {
      current_month = 8;
      current_date = current_day_of_current_year - 213;
    }
    else if ((current_day_of_current_year > 244) && (current_day_of_current_year <= 274)) {
      current_month = 9;
      current_date = current_day_of_current_year - 244;
    }
    else if ((current_day_of_current_year > 274) && (current_day_of_current_year <= 305)) {
      current_month = 10;
      current_date = current_day_of_current_year - 274;
    }
    else if ((current_day_of_current_year > 305) && (current_day_of_current_year <= 335)) {
      current_month = 11;
      current_date = current_day_of_current_year - 305;
    }
    else if (current_day_of_current_year > 335) {
      current_month = 12;
      current_date = current_day_of_current_year - 335;
    }
  } // END IF YEAR IS A LEAP YEAR
  
  number_of_seconds_in_current_day = number_of_seconds_of_current_year % number_of_seconds_in_a_day;
  current_hour = number_of_seconds_in_current_day / 3600;
  number_of_seconds_in_current_hour = number_of_seconds_in_current_day % 3600;
  current_minute = number_of_seconds_in_current_hour / 60;
  current_second = number_of_seconds_in_current_hour % 60;
  
  if (connection_established_between_zeo_and_arduino_program_flag == false) { // if this is the first time a time stamp has been received from Zeo's RTC
    
    digitalWrite(red_LED_pin, HIGH); // RED indicator LED ON
    digitalWrite(yellow_LED_pin, LOW); // YELLOW indicator LED OFF
    digitalWrite(green_LED_pin, LOW); // GREEN indicator LED OFF
    
    // turn RED indicator LED ON to indicate connection established between Zeo and Arduino and that user can undock headband
    
    current_month_tens_place = current_month / 10 ; // 1st character for file names: MMDD_XXX.csv
    current_month_ones_place = current_month % 10; // 2nd character for file names: MMDD_XXX.csv
    current_date_tens_place = current_date / 10; // 3rd character for file names: MMDD_XXX.csv
    current_date_ones_place = current_date % 10; // 4th character for file names: MMDD_XXX.csv

    current_month_tens_place_ascii_representation = current_month_tens_place + ascii_offset; // 1st character for file names: MMDD_XXX.csv
    current_month_ones_place_ascii_representation = current_month_ones_place  + ascii_offset; // 2nd character for file names: MMDD_XXX.csv
    current_date_tens_place_ascii_representation = current_date_tens_place + ascii_offset; // 3rd character for file names: MMDD_XXX.csv
    current_date_ones_place_ascii_representation = current_date_ones_place + ascii_offset; // 4th character for file names: MMDD_XXX.csv

    current_month_tens_place_for_file_name = char(current_month_tens_place_ascii_representation); // 1st character for file names: MMDD_XXX.csv
    current_month_ones_place_for_file_name = char(current_month_ones_place_ascii_representation); // 2nd character for file names: MMDD_XXX.csv
    current_date_tens_place_for_file_name = char(current_date_tens_place_ascii_representation); // 3rd character for file names: MMDD_XXX.csv
    current_date_ones_place_for_file_name = char(current_date_ones_place_ascii_representation); // 4th character for file names: MMDD_XXX.csv

    file_name_event_log[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_event_log[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_event_log[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_event_log[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    file_name_hypnogram[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_hypnogram[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_hypnogram[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_hypnogram[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    file_name_EEG[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_EEG[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_EEG[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_EEG[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    file_name_summary[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_summary[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_summary[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_summary[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    file_name_REM[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_REM[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_REM[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_REM[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    file_name_SQI[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_SQI[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_SQI[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_SQI[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array

    file_name_impedance[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_impedance[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_impedance[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_impedance[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array

    file_name_AWK[current_month_tens_place_file_name_element_number] = current_month_tens_place_for_file_name; // overwrite 0th element in array
    file_name_AWK[current_month_ones_place_file_name_element_number] = current_month_ones_place_for_file_name; // overwrite 1st element in array
    file_name_AWK[current_date_tens_place_file_name_element_number] = current_date_tens_place_for_file_name; // overwrite 2nd element in array
    file_name_AWK[current_date_ones_place_file_name_element_number] = current_date_ones_place_for_file_name; // overwrite 3rd element in array
    
    connection_established_between_zeo_and_arduino_program_flag = true; // set the flag so that this code block does not execute again
 
  } // end if
  
  return;
  
} // end of convert_from_unix_time() function

// ****************************************************************************************************************************************************

void headband_impedance() {

  impedance_across_headband_1 = zeo_data_stream_data_array[0]; // when data type is impedance, first item (element 0) in data stream is headband impedance value
  impedance_across_headband_2 = zeo_data_stream_data_array[1]; // when data type is impedance, second item (element 1) in data stream indicates whether headband connection is good

  if (file_impedance) { // if the file is available, write to it:
    
    file_impedance.print(current_year);
    file_impedance.print(","); // comma delimited fields
    file_impedance.print(current_month);
    file_impedance.print(","); // comma delimited fields
    file_impedance.print(current_date);
    file_impedance.print(","); // comma delimited fields
    file_impedance.print(current_hour);
    file_impedance.print(","); // comma delimited fields
    file_impedance.print(current_minute);
    file_impedance.print(","); // comma delimited fields
    file_impedance.print(current_second);
    file_impedance.print(","); // comma delimited fields
    
    file_impedance.print(impedance_across_headband_1);
    file_impedance.print(","); // comma delimited fields
    file_impedance.println(impedance_across_headband_2); // last item written to file must be printed using "println" to move to next line

  } // end if  
  
  return;
  
} // end of headband_impedance() function

// ****************************************************************************************************************************************************

void sleep_stage() {
  
  sleep_stage_type = zeo_data_stream_data_array[0]; // when data type is sleep stage, first item (element 0) in data stream is the sleep stage value
  
  if ((sleep_stage_type_old == sleep_stage_rem) && (sleep_stage_type != sleep_stage_rem)) { // if last stage was rem and current stage is not rem, then rem epoch has ended
  
    number_of_rem_epochs = number_of_rem_epochs + 1; // another instance of rem or epoch has occurred (distinct and not contiguous events)

    rem_epoch_duration_minutes = rem_epoch_duration_seconds / number_of_seconds_in_a_minute; // covert from seconds to minutes
  
    if (file_REM) { // if the file opened okay, write to it:
      
      file_REM.print(number_of_rem_epochs); // epoch has to begin before it ends so first epoch will appear as 1 not 0
      file_REM.print(","); // comma delimited fields
      file_REM.println(rem_epoch_duration_minutes); // last item written to file must be printed using "println" to move to next line
   
    } // end if
    
    rem_epoch_duration_seconds = 0; // rem epoch has ended, log it and reset the epoch duration counter to 0
    
  } // end if
    
  if (sleep_onset_program_flag == true) { // only execute if sleep onset has occurred: prevents time to fall asleep from being recorded as an awakening
    
    if ((sleep_stage_type_old == sleep_stage_awake) && (sleep_stage_type != sleep_stage_awake)) { // if last stage was awake and current stage is not awake, then awakening has ended
  
      number_of_awakenings = number_of_awakenings + 1; // another awakening has occurred (distinct and not contiguous events)

      awakening_duration_minutes = awakening_duration_seconds / number_of_seconds_in_a_minute; // covert from seconds to minutes
  
      if (file_AWK) { // if the file opened okay, write to it:
      
        file_AWK.print(number_of_awakenings); // awakening has to begin before it ends so first epoch will appear as 1 not 0
        file_AWK.print(","); // comma delimited fields
        file_AWK.println(awakening_duration_minutes); // last item written to file must be printed using "println" to move to next line
   
      } // end if
    
      wakefulness_after_sleep_onset_minutes = wakefulness_after_sleep_onset_minutes + awakening_duration_minutes; // increment (time between sleep onset and final awakening)
     
      awakening_duration_seconds = 0; // awakening has ended, log it and reset the awakening duration counter to 0
    
    } // end if
    
  } // end if
  
  switch (sleep_stage_type) {
    case sleep_stage_undefined:
      sleep_stage_message = sleep_stage_message_undefined;
      total_seconds_unknown_sleep_stage = total_seconds_unknown_sleep_stage + 30; // sleep stage determination is made every 30 seconds
      break; // break out of selection process otherwise command under next case will execute   
    case sleep_stage_awake:
      sleep_stage_message = sleep_stage_message_awake;
      total_seconds_awake_sleep_stage = total_seconds_awake_sleep_stage + 30; // sleep stage determination is made every 30 seconds
      if (sleep_onset_program_flag == true) { // only execute if sleep onset has occurred: prevents time to fall asleep from being recorded as an awakening
        awakening_duration_seconds = awakening_duration_seconds + 30; // sleep stage determination is made every 30 seconds
      } // end if
      break; // break out of selection process otherwise command under next case will execute   
    case sleep_stage_rem:
      sleep_stage_message = sleep_stage_message_rem;
      total_seconds_rem_sleep_stage = total_seconds_rem_sleep_stage + 30; // sleep stage determination is made every 30 seconds
      total_sleep_time_seconds = total_sleep_time_seconds + 30; // sleep stage determination is made every 30 seconds (asleep = L, D, or REM)
      if (rem_onset_program_flag == false) { // if this is the first time REM has occurred
        unix_time_seconds_since_unix_epoch_rem_onset = unix_time_seconds_since_unix_epoch; // time of rem onset set to most recent time stamp from RTC
        rem_onset_program_flag = true; // set the flag so that this code block does not execute again
      } // end if
      rem_epoch_duration_seconds = rem_epoch_duration_seconds + 30; // sleep stage determination is made every 30 seconds
      break;  // break out of selection process otherwise command under next case will execute   
    case sleep_stage_light:
      sleep_stage_message = sleep_stage_message_light;
      total_seconds_light_sleep_stage = total_seconds_light_sleep_stage + 30; // sleep stage determination is made every 30 seconds
      total_sleep_time_seconds = total_sleep_time_seconds + 30; // sleep stage determination is made every 30 seconds (asleep = L, D, or REM)
      break;  // break out of selection process otherwise command under next case will execute   
    case sleep_stage_deep:
      sleep_stage_message = sleep_stage_message_deep;
      total_seconds_deep_sleep_stage = total_seconds_deep_sleep_stage + 30; // sleep stage determination is made every 30 seconds
      total_sleep_time_seconds = total_sleep_time_seconds + 30; // sleep stage determination is made every 30 seconds (asleep = L, D, or REM)
      break; // break out of selection process otherwise command under next case will execute   
  } // end switch
  
  if ((sleep_stage_type == sleep_stage_rem) || (sleep_stage_type == sleep_stage_light) || (sleep_stage_type == sleep_stage_deep)) {
    if (sleep_onset_program_flag == false) { // if this is the first time sleep onset event has occurred (the first epoch of sleep no matter what stage) 
      unix_time_seconds_since_unix_epoch_sleep_onset = unix_time_seconds_since_unix_epoch; // time of sleep onset set to most recent time stamp from RTC
      sleep_onset_program_flag = true; // set the flag so that this code block does not execute again
    } // end if
  } // end if
 
  sleep_stage_type_5s_complement_for_hypnogram_reconstruction = 5 - sleep_stage_type; // take 5's complement of zeo sleep stage to get value for reconstructing hypnogram
 
  if (file_hypnogram) { // if the file opened okay, write to it:
  
    file_hypnogram.print(current_year);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(current_month);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(current_date);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(current_hour);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(current_minute);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(current_second);
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(sleep_stage_type); // NUMBER 0 - 4 FOR CONSTRUCTING HYPNOGRAM
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.print(sleep_stage_type_5s_complement_for_hypnogram_reconstruction); // NUMBER 0 - 5 FOR CONSTRUCTING HYPNOGRAM WITH CONVENTION TO FACILITATE COMPARISON TO PSG
    file_hypnogram.print(","); // comma delimited fields
    file_hypnogram.println(sleep_stage_message); // last item written to file must be printed using "println" to move to next line
   
  } // end if 

  sleep_stage_type_old = sleep_stage_type; // update old sleep stage
  
  return;
  
} // end of sleep_stage() function

// ****************************************************************************************************************************************************

void create_and_open_files() {
  
  if (SD.exists(file_name_event_log)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_event_log); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_event_log)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if  
  
  if (SD.exists(file_name_hypnogram)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_hypnogram); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_hypnogram)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
  if (SD.exists(file_name_EEG)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_EEG); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_EEG)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
  if (SD.exists(file_name_summary)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_summary); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_summary)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
   if (SD.exists(file_name_REM)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_REM); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_REM)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
   if (SD.exists(file_name_SQI)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_SQI); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_SQI)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if

   if (SD.exists(file_name_impedance)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_impedance); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_impedance)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
  if (SD.exists(file_name_AWK)) { // 8.3 FILE NAMING FORMAT
    do {
      SD.remove(file_name_AWK); // if file is already present on sd card, erase it
    } while (SD.exists(file_name_AWK)); // neccessary to confirm file erasure was successful as sometimes it isn't which results in multiple runs appearing in the same file (appeneded)
  } // end if
  
  do { // execute the code block at least one time:
    file_event_log = SD.open(file_name_event_log, FILE_WRITE); // open file for writing
  } while (!file_event_log); // keep trying to open file until it opens
  
  if (file_event_log) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_event_log.println("EVENT LOG FILE");
    file_event_log.println(" "); // blank line
    file_event_log.println("YEAR" "," "MONTH" "," "DATE" "," "HOUR" "," "MIN" "," "SEC" "," "EVENT"); 
    file_event_log.println(" "); // blank line, last item written to file must be printed using "println" to move to next line

    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
  
  } // end if
  
  do { // execute the code block at least one time:
    file_hypnogram = SD.open(file_name_hypnogram, FILE_WRITE); // open file for writing
  } while (!file_hypnogram); // keep trying to open file until it opens
 
  if (file_hypnogram) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_hypnogram.println("SLEEP STAGE FILE");
    file_hypnogram.println(" "); // blank line
    file_hypnogram.println("YEAR" "," "MONTH" "," "DATE" "," "HOUR" "," "MIN" "," "SEC" "," "ZEO SLEEP STAGE" "," "SLEEP STAGE FOR HYPNOGRAM" "," "DESCRIPTION"); 
    file_hypnogram.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
    
  } // end if
  
  do { // execute the code block at least one time:
    file_EEG = SD.open(file_name_EEG, FILE_WRITE); // open file for writing
  } while (!file_EEG); // keep trying to open file until it opens
  
  if (file_EEG) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_EEG.println("EEG DATA FILE");
    file_EEG.println(" "); // blank line
    file_EEG.println("YEAR" "," "MONTH" "," "DATE" "," "HOUR" "," "MIN" "," "SEC" "," "DELTA (2-4 Hz)" "," "THETA (4-8 Hz)" "," "ALPHA (8-13 Hz)" "," "BETA (13-18 Hz)" "," "BETA (18-21 Hz)" "," "BETA - SLEEP SPINDLES (11-14 Hz)" "," "GAMMA (30-50 Hz)" "," "EEG ACTIVITY (TOTAL)" "," "DELTA % (2-4 Hz)" "," "THETA % (4-8 Hz)" "," "ALPHA % (8-13 Hz)" "," "BETA % (13-18 Hz)" "," "BETA % (18-21 Hz)" "," "BETA - SLEEP SPINDLES % (11-14 Hz)" "," "GAMMA % (30-50 Hz)"); 
    file_EEG.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if
  
  do { // execute the code block at least one time:
    file_night_summary = SD.open(file_name_summary, FILE_WRITE); // open file for writing
  } while (!file_night_summary); // keep trying to open file until it opens
  
  if (file_night_summary) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_night_summary.println("NIGHT SUMMARY FILE");
    file_night_summary.println(" "); // blank line
    file_night_summary.println("TRT (MIN)" "," "TST (MIN)" "," "TWT (MIN)" "," "SE (%)" "," "TOTAL UNK (MIN)" "," "TOTAL L (MIN)" "," "TOTAL D (MIN)" "," "TOTAL REM (MIN)" "," "% L" "," "% D" "," "% REM" "," "SOL (MIN)" "," "LPS (MIN)" "," "WASO (MIN)" "," "NUMBER OF AWAKENINGS" "," "REM ONSET LATENCY (MIN)" "," "NUMBER OF REM EPOCHS"); 
    file_night_summary.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if
  
  do { // execute the code block at least one time:
    file_REM = SD.open(file_name_REM, FILE_WRITE); // open file for writing
  } while (!file_REM); // keep trying to open file until it opens
  
  if (file_REM) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_REM.println("REM FILE");
    file_REM.println(" "); // blank line
    file_REM.println("REM EPOCH NUMBER" "," "EPOCH DURATION (MIN)"); 
    file_REM.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if

  do { // execute the code block at least one time:
    file_SQI = SD.open(file_name_SQI, FILE_WRITE); // open file for writing
  } while (!file_SQI); // keep trying to open file until it opens
  
  if (file_SQI) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_SQI.println("SQI FILE");
    file_SQI.println(" "); // blank line
    file_SQI.println("YEAR" "," "MONTH" "," "DATE" "," "HOUR" "," "MIN" "," "SEC" "," "SQI"); 
    file_SQI.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if

  do { // execute the code block at least one time:
    file_impedance = SD.open(file_name_impedance, FILE_WRITE); // open file for writing
  } while (!file_impedance); // keep trying to open file until it opens
  
  if (file_impedance) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_impedance.println("IMPEDANCE FILE");
    file_impedance.println(" "); // blank line
    file_impedance.println("YEAR" "," "MONTH" "," "DATE" "," "HOUR" "," "MIN" "," "SEC" "," "IMP 1" "," "IMP 2"); 
    file_impedance.println(" "); // blank line, last item written to file must be printed using "println" to move to next line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if   
 
  do { // execute the code block at least one time:
    file_AWK = SD.open(file_name_AWK, FILE_WRITE); // open file for writing
  } while (!file_AWK); // keep trying to open file until it opens
  
  if (file_AWK) { // if the file opened okay, write to it (evaluates as TRUE if open)
  
    file_AWK.println("AWK FILE");
    file_AWK.println(" "); // blank line
    file_AWK.println("AWAKENING NUMBER" "," "AWAKENING DURATION (MIN)"); 
    file_AWK.println(" "); // blank line
    
    // don't close file - keep it open for logging data to it - close it at end of program when headband is redocked to base 
     
  } // end if     
  
  digitalWrite(red_LED_pin, LOW); // RED indicator LED OFF
  digitalWrite(yellow_LED_pin, HIGH); // YELLOW indicator LED ON
  digitalWrite(green_LED_pin, LOW); // GREEN indicator LED OFF
  
  // files created and opened, data being logged to files
  
  // as of version 1.0, it is possible to have multiple files open: keep files open to reduce write time and close at end when night concludes and headband is redocked
  
  return;
  
} // end of create_and_open_files() function

// ****************************************************************************************************************************************************
