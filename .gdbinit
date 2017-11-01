target ext :2331
mon endian little

define restart
	mon halt
	load
	mon reset
end


define l
	layout next
end


restart
l
