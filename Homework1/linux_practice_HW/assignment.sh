#!/bin/bash
mkdir -p linux_practice/docs linux_practice/backup
touch linux_practice/docs/readme.txt linux_practice/docs/notes.log linux_practice/docs/temp.tmp
rm linux_practice/docs/temp.tmp
mv linux_practice/docs/notes.log linux_practice/docs/daily_report.txt
echo "Project Status: Active" > linux_practice/docs/daily_report.txt
date >> linux_practice/docs/daily_report.txt
cp linux_practice/docs/*.txt linux_practice/backup/
chmod 444 linux_practice/backup/*.txt

for file in linux_practice/backup/*.txt; do
	filesname=$(basename "$file")
	echo "Archive Complete. File [ $filename ] is now read-only."
done
