sudo adduser amigo audio
sudo adduser amigo pulse
sudo adduser amigo pulse-access

red='\033[0;31m'
NC='\033[0m' # No Color
echo -e "${red}\nNow check if the following commands work:\n - mpg123 -o pulse <mp3-file>\n - arecord <wav-file>\n - aplay <wav-file>\n${NC}"
