#!/bin/ruby
require 'csv'

#�t�@�C���e�X�g
if File.exist?(ARGV[0]) == false then
	print "file not found\n"
	exit 1
end

#CSV��ǂݍ���ŁA16�i����10�i���ϊ����s��
CSV.foreach(ARGV[0]) do |r|
	r.each do |v|
		print v.hex.to_s() + ", "
	end
	print "\n"
end
