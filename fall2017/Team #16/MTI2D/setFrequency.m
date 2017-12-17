function setFrequency( mti2d, freq )
%100Hz - 6000Hz
acql_t = 1/freq*10^6;
mti2d.setAcquisitionLineTime(int2str(acql_t));

end

