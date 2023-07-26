# pip install pillow
# pip install pytesseract

import pytesseract
import cv2 as cv
from PIL import Image

# install tesseract and set environment variable path 
pytesseract.pytesseract.tesseract_cmd = r'C:\Users\Satyajeet\AppData\Local\Tesseract-OCR\tesseract.exe'
img = Image.open('text.png')
print(pytesseract.image_to_string(img))