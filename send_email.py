my_gmail_address = ""
password = ""

import sys
import smtplib
import time
from email.MIMEMultipart import MIMEMultipart
from email.MIMEText import MIMEText
from email.MIMEBase import MIMEBase
from email import encoders

log_name = sys.argv[1] 

fromaddr = my_gmail_address
toaddr = my_gmail_address
 
msg = MIMEMultipart()
 
msg['From'] = fromaddr
msg['To'] = toaddr
msg['Subject'] = "Espresso logs (" + time.strftime('%Y-%m-%d at %H:%M') + ")"
 
body = "Graph & flow data attached"
# Add more extraction parameters here?
 
msg.attach(MIMEText(body, 'plain'))
 
filename = "graph-" + log_name + ".pdf"
path = "/home/pi/logs/"

def create_attachment(path, filename):
    attachment = open(path + filename, "rb")
    part = MIMEBase('application', 'octet-stream')
    part.set_payload((attachment).read())
    encoders.encode_base64(part)
    part.add_header('Content-Disposition', "attachment; filename= %s" % filename)
    return part
 
part1 = create_attachment("/home/pi/logs/", "graph-" + log_name + ".pdf")
part2 = create_attachment("/home/pi/logs/", log_name + ".json")

msg.attach(part1)
msg.attach(part2)

server = smtplib.SMTP('smtp.gmail.com', 587)
server.starttls()
server.login(fromaddr, "")
text = msg.as_string()
server.sendmail(fromaddr, toaddr, text)
server.quit()