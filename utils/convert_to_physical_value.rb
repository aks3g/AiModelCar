#!/bin/ruby
require 'csv'

def convertToSigned16bit(v)
	unsignedVal = v.hex;

	if unsignedVal >"7FFF".hex then
		return unsignedVal - "10000".hex;
	else
		return unsignedVal
	end
end


def convertToSigned32bit(v)
	unsignedVal = v.hex;
	if unsignedVal >"7FFFFFFF".hex then
		return unsignedVal - "100000000".hex;
	else
		return unsignedVal
	end
end


#�t�@�C���e�X�g
if File.exist?(ARGV[0]) == false then
	print "file not found\n"
	exit 1
end

#CSV��ǂݍ���ŁA�t�B�[���h�ɉ�����
CSV.foreach(ARGV[0]) do |r|
	# ����
	print r[0].hex.to_s() + ", "

	# ����
	print convertToSigned32bit(r[1]).to_s() + ", "

	# Duty
	print r[2].hex.to_s() + ", "

	# �����xX
	print convertToSigned16bit(r[3]).to_s() + ", "

	# �����xY
	print convertToSigned16bit(r[4]).to_s() + ", "

	# �����xZ
	print convertToSigned16bit(r[5]).to_s() + ", "

	# �p���xX
	print convertToSigned16bit(r[6]).to_s() + ", "

	# �p���xY
	print convertToSigned16bit(r[7]).to_s() + ", "

	# �p���xZ
	print convertToSigned16bit(r[8]).to_s() + "\n"
end


