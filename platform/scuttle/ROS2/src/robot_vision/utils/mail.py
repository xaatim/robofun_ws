from email.message import EmailMessage
import smtplib
import ssl
import os

def send_mail(image_path):
  EMAIL_SENDER = "xayari229@gmail.com"
  EMAIL_PASSWORD = "rkoa zvwu nqnj nifo"
  EMAIL_RECEIVER = "xayari229@gmail.com"
  context = ssl.create_default_context()
  try:
      msg = EmailMessage()
      msg["Subject"] = "Intruder Alert!"
      msg["From"] = EMAIL_SENDER
      msg["To"] = EMAIL_RECEIVER
      msg.set_content("An unknown person was detected while you were away. Image attached.")

      with open(image_path, "rb") as f:
          img_data = f.read()
          msg.add_attachment(img_data, maintype='image', subtype='jpeg', filename="intruder.jpg")

      with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as smtp:
          smtp.login(EMAIL_SENDER, EMAIL_PASSWORD)
          smtp.send_message(msg)


      # Delete image after successful email
      os.remove(image_path)
      print(f"Deleted alert image: {image_path}")

  except Exception as e:
      print(f"Error sending email: {e}")