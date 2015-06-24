#!/usr/bin/env ruby

require 'rexml/document'

xmlInput = File.read(ARGV[0])
csvOutputPrefix = ARGV[1]

doc, arrayOfHashes = REXML::Document.new(xmlInput), []
doc.elements.each('testsuites/testsuite/testcase') do |t|
  arrayOfHashes << t.attributes
end
sortedKeys = arrayOfHashes.first.keys.sort
sortedKeys.delete("value_param")

timestamp = doc.elements.first.attributes["timestamp"]
csvOutput = csvOutputPrefix + "_" + timestamp + ".csv"
File.open(csvOutput, "w") do |f|
  f.puts sortedKeys.join(',')
  arrayOfHashes.each do |h|
    row = []
    sortedKeys.each do |k|
      row << h[k]
    end
    f.puts row.join(',')
  end
end
