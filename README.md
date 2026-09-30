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
