#!/bin/ruby
require 'csv'

#ファイルテスト
if File.exist?(ARGV[0]) == false then
	print "file not found\n"
	exit 1
end

#CSVを読み込んで、16進数→10進数変換を行う
CSV.foreach(ARGV[0]) do |r|
	r.each do |v|
		print v.hex.to_s() + ", "
	end
	print "\n"
end
