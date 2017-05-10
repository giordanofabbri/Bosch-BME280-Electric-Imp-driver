class BME280 {
      Registers = {
        DIG_T1              = 0x88
        DIG_T2              = 0x8A
        DIG_T3              = 0x8C
    
        DIG_P1              = 0x8E
        DIG_P2              = 0x90
        DIG_P3              = 0x92
        DIG_P4              = 0x94
        DIG_P5              = 0x96
        DIG_P6              = 0x98
        DIG_P7              = 0x9A
        DIG_P8              = 0x9C
        DIG_P9              = 0x9E
    
        DIG_H1              = 0xA1
        DIG_H2              = 0xE1
        DIG_H3              = 0xE3
        DIG_H4              = 0xE4
        DIG_H5              = 0xE5
        DIG_H6              = 0xE7
    
        CHIPID             = 0xD0
        VERSION            = 0xD1
        SOFTRESET          = 0xE0
    
        CAL26              = 0xE1
    
        CONTROLHUMID       = 0xF2
        STATUS             = 0xF3
        MEASURE            = 0xF4
        CONFIG             = 0xF5
        PRESSUREDATA       = 0xF7
        TEMPDATA           = 0xFA
        HUMIDDATA          = 0xFD
    };    
      SamplingValues = {
        NONE = 0x00
        X1   = 0x01
        X2   = 0x02
        X4   = 0x03
        X8   = 0x04
        X16  = 0x05
    };
      Modes = {
        SLEEP   = 0x00
        FORCED   = 0x01
        NORMAL   = 0x02
    };
      StandByValues = {
        MS_0_5  = 0x00
        MS_62_5 = 0x01
        MS_125  = 0x02
        MS_250  = 0x03
        MS_500  = 0x04
        MS_1000 = 0x05
        MS_10   = 0x06
        MS_20   = 0x07
    };
      FilterValues = {
        OFF = 0x00
        X2  = 0x01
        X4  = 0x02
        X8  = 0x03
        X16 = 0x04
      };
      SPIModes = {
        WIRE4 = 0x00
        WIRE3 = 0x01
      };
    
	  _i2c = null;
	  _addr = 0;
	  t_fine = 0;

	  constructor(i2cbus, addr_7bit) {
		  _i2c = i2cbus;
		  _addr = addr_7bit << 1;
	  }

	  function _readReg(register, numBytes) {
  		local result = _i2c.read(_addr, register, numBytes);
  		if (result == null) {
  			throw "I2C read error: " + _i2c.readerror();
  		}
  		return result;
	  }

	  function _writeReg(register, data) {
  		local reg_data = register + data;
  		local result = _i2c.write(_addr, reg_data);
  		if (result != 0) {
  			throw "I2C write error: " + result;
 		}
  		return result;
	  }
	  
	  function _read2BytesLEUnsigned(register){
	    local rawRead = _readReg(register, 2);
	    local retVal = rawRead[1];
	    return (retVal << 8) + rawRead[0];
	  }

	  function _read2BytesLESigned(register){
	    local rawRead = _readReg(register, 2);
	    local retVal = rawRead[1];
	    retVal = (retVal << 8) + rawRead[0];
	    return (retVal <<16) >> 16;
	  }

	  function _readUnsignedChar(register){
	    local rawRead = _readReg(register, 1);
	    return rawRead[0];
	  }

	  function _readSignedChar(register){
	    local rawRead = _readReg(register, 1);
	    return (rawRead[0] << 24) >> 24;
	  }

	  function CheckChipAddress() {
  		local i2cByte;
  		i2cByte = _readReg(Registers.CHIPID.tochar(), 1)[0];
  		return i2cByte == 0x60;
	  }

	  function SetSamplingMode(){
	        local t_samplingValue = SamplingValues.X1;
	        local p_samplingValue = SamplingValues.X1;
	        local modeValue = Modes.FORCED;
	        local measureRegValue = (t_samplingValue << 5) + (p_samplingValue << 3) + modeValue;

	        local standByValue = StandByValues.MS_1000;
	        local filterValue = FilterValues.OFF;
	        local spi3wEnable = SPIModes.WIRE3;

	        local configRegValue = (standByValue << 5) + (filterValue << 3) + spi3wEnable;

  			_writeReg(Registers.CONFIG.tochar(), configRegValue.tochar()); 
  			_writeReg(Registers.CONTROLHUMID.tochar(), SamplingValues.X1.tochar()); 
  			_writeReg(Registers.MEASURE.tochar(), measureRegValue.tochar()); 
	  }
	  
	  function WaitForChipReady(){
        local busy = 1;
        do{
            imp.sleep(0.01);
            local statusreg = _readReg(Registers.STATUS.tochar(), 1);
	        busy = statusreg[0] & 0x01;
            }while (busy != 0x00);
	  }

	  function WaitForMeasureCompleted(){
        local busy = 1;
        do{
            imp.sleep(0.01);
            local statusreg = _readReg(Registers.STATUS.tochar(), 1);
	        busy = statusreg[0] & 0x08;
            }while (busy != 0x00);
	  }

	  function init() {

        WaitForChipReady();	    

  		if (!CheckChipAddress()) {
  			server.log(format("Invalid chip address: 0x%02X", _addr));
  		} else {
  			server.log(format("BME280 chip address: 0x%02X", _addr));
  			_writeReg(Registers.SOFTRESET.tochar(), "\xB6"); 
	        WaitForChipReady();	    
  		}
	  }
	  
	  function _readRawTemperature(){
	    local regData = _readReg(Registers.TEMPDATA.tochar(), 3);
	    local value = regData[0];
	    value = value << 8;
	    value = value + regData[1];
	    value = value << 8;
	    value = value + regData[2];
	    return value >> 4;
	  }
	  
	  function _compensateRawTemperature(adc_T){
	    local dig_T1 = _read2BytesLEUnsigned(Registers.DIG_T1.tochar());
	    local dig_T2 = _read2BytesLESigned(Registers.DIG_T2.tochar());
	    local dig_T3 = _read2BytesLESigned(Registers.DIG_T3.tochar());

	    local var1  = ((((adc_T>>3) - (dig_T1 <<1))) * (dig_T2)) >> 11;
        local var2  = (((((adc_T>>4) - (dig_T1)) * ((adc_T>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;

        t_fine = var1 + var2;

	    local T = (t_fine * 5 + 128) >> 8;
	    T=T/100.0;
	    
	    return T;
	  }
	  
	  function readTemperature() {
	    local adc_T = _readRawTemperature();  
        return _compensateRawTemperature(adc_T);
	  }

      function _readRawPressure(){
	    local regData = _readReg(Registers.PRESSUREDATA.tochar(), 3);
	    
	    local value = regData[0];
	    value = value << 8;
	    value = value + regData[1];

	    value = value << 8;
	    value = value + regData[2];
	    return value >> 4;
      }

      function _compensateRawPressureFixedPoint(adc_P){
          
	    local dig_P1 = _read2BytesLEUnsigned(Registers.DIG_P1.tochar());
	    local dig_P2 = _read2BytesLESigned(Registers.DIG_P2.tochar());
	    local dig_P3 = _read2BytesLESigned(Registers.DIG_P3.tochar());
	    local dig_P4 = _read2BytesLESigned(Registers.DIG_P4.tochar());
	    local dig_P5 = _read2BytesLESigned(Registers.DIG_P5.tochar());
	    local dig_P6 = _read2BytesLESigned(Registers.DIG_P6.tochar());
	    local dig_P7 = _read2BytesLESigned(Registers.DIG_P7.tochar());
	    local dig_P8 = _read2BytesLESigned(Registers.DIG_P8.tochar());
	    local dig_P9 = _read2BytesLESigned(Registers.DIG_P9.tochar());
          
        local p =0;
        local var1 = (t_fine>>1) - 64000 ;
        local var2 = (((var1>>2) * (var1>>2)) >> 11 ) * (dig_P6);
        var2 = var2 + ((var1*(dig_P5))<<1);
        var2 = (var2>>2)+((dig_P4)<<16);
        var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + (((dig_P2) * var1)>>1))>>18;
        var1 =((((32768+var1))*(dig_P1))>>15);
        if (var1 == 0)
        {
                return 0; // avoid exception caused by division by zero
        }
        p = ((((1048576)-adc_P)-(var2>>12)))*3125;
        if (p < 0x80000000)
        {
            p = (p << 1) / (var1);
        }
        else
        {
            p = (p / var1) * 2;
        }
        var1 = ((dig_P9) * ((((p>>3) * (p>>3))>>13)))>>12;
        var2 = (((p>>2)) * (dig_P8))>>13;
        p = (p + ((var1 + var2 + dig_P7) >> 4));
    
        return p;          
      }
      
      function _compensateRawPressureFloatingPoint(adc_P){
          
	    local dig_P1 = _read2BytesLEUnsigned(Registers.DIG_P1.tochar());
	    local dig_P2 = _read2BytesLESigned(Registers.DIG_P2.tochar());
	    local dig_P3 = _read2BytesLESigned(Registers.DIG_P3.tochar());
	    local dig_P4 = _read2BytesLESigned(Registers.DIG_P4.tochar());
	    local dig_P5 = _read2BytesLESigned(Registers.DIG_P5.tochar());
	    local dig_P6 = _read2BytesLESigned(Registers.DIG_P6.tochar());
	    local dig_P7 = _read2BytesLESigned(Registers.DIG_P7.tochar());
	    local dig_P8 = _read2BytesLESigned(Registers.DIG_P8.tochar());
	    local dig_P9 = _read2BytesLESigned(Registers.DIG_P9.tochar());
          
        local var1, var2, p;
        var1 = (t_fine/2.0) - 64000.0;
        var2 = var1 * var1 * (dig_P6) / 32768.0;
        var2 = var2 + var1 * (dig_P5) * 2.0;
        var2 = (var2/4.0)+((dig_P4) * 65536.0);
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + (dig_P2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0)*(dig_P1);
        if (var1 == 0.0)        {
            return 0; // avoid exception caused by division by zero
        }
        p = 1048576.0 - adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = (dig_P9) * p * p / 2147483648.0;
        var2 = p * (dig_P8) / 32768.0;
        p = p + (var1 + var2 + (dig_P7)) / 16.0;
        return p;        
      }

	  function readPressure() {
	    local adc_P = _readRawPressure();
	    return _compensateRawPressureFixedPoint(adc_P);
	  }
	  
  	  function _read_digH4(){
	    local rawRead = _readReg(Registers.DIG_H4.tochar(), 2);
	    local retVal = rawRead[1] & 0x0F;
	    retVal = retVal + (rawRead[0] << 4);
	    return (retVal <<20) >> 20;
	  }

	  function _read_digH5(){
	    local rawRead = _readReg(Registers.DIG_H5.tochar(), 2);
	    local retVal = rawRead[0] & 0x0F;
	    retVal = retVal + (rawRead[1] << 4);
	    return (retVal <<20) >> 20;
	  }

      function _readRawHumidity(){
	    local regData = _readReg(Registers.HUMIDDATA.tochar(), 2);
	    local value = regData[0];
	    value = value << 8;
	    return value + regData[1];
      }
      
      function _compensateRawHumidityFixedPoint(adc_H){
	    local dig_H1 = _readUnsignedChar(Registers.DIG_H1.tochar());
	    local dig_H2 = _read2BytesLESigned(Registers.DIG_H2.tochar());
	    local dig_H3 = _readUnsignedChar(Registers.DIG_H3.tochar());
	    local dig_H4 = _read_digH4();
	    local dig_H5 = _read_digH5();
	    local dig_H6 = _readSignedChar(Registers.DIG_H6.tochar());
                  
        local v_x1_u32r;
        v_x1_u32r = (t_fine - (76800));
        v_x1_u32r = (((((adc_H << 14) - ((dig_H4) << 20) - ((dig_H5) * v_x1_u32r)) +
        (16384)) >> 15) * (((((((v_x1_u32r * (dig_H6)) >> 10) * (((v_x1_u32r *
        (dig_H3)) >> 11) + (32768))) >> 10) + (2097152)) *
        (dig_H2) + 8192) >> 14));
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (dig_H1)) >> 4));
        
        if(v_x1_u32r < 0){
            v_x1_u32r = 0;    
        }
        
        if(v_x1_u32r > 419430400){
            v_x1_u32r = 419430400;
        }
        
        local h = (v_x1_u32r>>12) / 1024;
        return h;      
      }
      
      function _compensateRawHumidityFloatingPoint(adc_H){
	    local dig_H1 = _readUnsignedChar(Registers.DIG_H1.tochar());
	    local dig_H2 = _read2BytesLESigned(Registers.DIG_H2.tochar());
	    local dig_H3 = _readUnsignedChar(Registers.DIG_H3.tochar());
	    local dig_H4 = _read_digH4();
	    local dig_H5 = _read_digH5();
	    local dig_H6 = _readSignedChar(Registers.DIG_H6.tochar());

        local var_H;
        var_H = ((t_fine) - 76800.0);
        var_H = (adc_H - ((dig_H4) * 64.0 + (dig_H5) / 16384.0 * var_H)) *
        ((dig_H2) / 65536.0 * (1.0 + (dig_H6) / 67108864.0 * var_H *
        (1.0 + (dig_H3) / 67108864.0 * var_H)));
        var_H = var_H * (1.0 - (dig_H1) * var_H / 524288.0);
        
        if (var_H > 100.0)
            var_H = 100.0;
        else if (var_H < 0.0)
            var_H = 0.0;
        
        return var_H;        
      }

      function readHumidity(){
        local adc_H = _readRawHumidity();
        return _compensateRawHumidityFloatingPoint(adc_H);
      }
}
