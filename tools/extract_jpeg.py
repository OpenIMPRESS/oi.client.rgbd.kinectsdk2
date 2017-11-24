import sys
import os
import struct

def main():
	readFile(sys.argv[1])

def readFile(name):
	dir_name = name+"_frames"
	directory = os.path.dirname(dir_name)
	if not os.path.exists(directory):
		os.makedirs(dir_name)
	with open(name, "rb") as f:
		frame_nr = 0;
		while True: 
			data_len_bytes = f.read(4)
			if len(data_len_bytes) < 4: break
			data_len = struct.unpack('I', data_len_bytes)[0];
			img_buf = f.read(data_len)
			writeFrame(dir_name, name, frame_nr, img_buf)
			frame_nr = frame_nr+1
		

def writeFrame(dir_name, name, frame, buffer):
	f = open(dir_name+"/"+name+"_frame_"+str(frame)+".jpg","wb") 
	f.write(buffer)
	f.close()
	print("Frame "+str(frame)+" extracted.");
	return
	
if __name__ == "__main__": main()
