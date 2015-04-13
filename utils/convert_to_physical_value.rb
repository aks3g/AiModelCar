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


#ファイルテスト
if File.exist?(ARGV[0]) == false then
	print "file not found\n"
	exit 1
end

#CSVを読み込んで、フィールドに応じた
CSV.foreach(ARGV[0]) do |r|
	# 時刻
	print r[0].hex.to_s() + ", "

	# 距離
	print convertToSigned32bit(r[1]).to_s() + ", "

	# Duty
	print r[2].hex.to_s() + ", "

	# 加速度X
	print convertToSigned16bit(r[3]).to_s() + ", "

	# 加速度Y
	print convertToSigned16bit(r[4]).to_s() + ", "

	# 加速度Z
	print convertToSigned16bit(r[5]).to_s() + ", "

	# 角速度X
	print convertToSigned16bit(r[6]).to_s() + ", "

	# 角速度Y
	print convertToSigned16bit(r[7]).to_s() + ", "

	# 角速度Z
	print convertToSigned16bit(r[8]).to_s() + "\n"
end


