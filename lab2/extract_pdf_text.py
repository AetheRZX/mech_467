import pypdf
import sys

def extract_text(pdf_path, output_path):
    try:
        reader = pypdf.PdfReader(pdf_path)
        text = ""
        for page in reader.pages:
            text += page.extract_text() + "\n"
        
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"Successfully wrote to {output_path}")
    except Exception as e:
        print(str(e))

if __name__ == "__main__":
    if len(sys.argv) > 2:
        pdf_path = sys.argv[1]
        output_path = sys.argv[2]
        extract_text(pdf_path, output_path)
    else:
        print("Usage: python extract_pdf_text.py <input_pdf> <output_txt>")
