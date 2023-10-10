from flask import Flask, render_template, request
import json

app = Flask(__name__)

# Serve the HTML page with the file upload form
@app.route('/')
def index():
    # Display initial UI
    return render_template("index.html")


# Handle the JSON file upload and processing
@app.route('/read_json', methods=['POST'])
def read_json():
    # Check if a file was included in the request
    # The 'json_file' is a reference to the file input field in the html file.
    if 'json_file' not in request.files:
        return 'No JSON file uploaded'
    file = request.files['json_file']
    # Check if the file has a valid JSON extension
    if file.filename == '' or not file.filename.endswith('.json'):
        return 'Invalid JSON file'

    try:
        # Load and parse the JSON data
        json_data = json.load(file)
        # Initialize dictionary containing name of each visualization attribute in json file with a True (checkbox value)
        names = {item["name"]: True for item in json_data["data"]}   

        for obj_name in names:
            names[obj_name] = bool(request.form.get(obj_name, False))

        return render_template("read_json.html", names = names)
    except json.JSONDecodeError:
        return 'Invalid JSON format'

if __name__ == '__main__':
    app.run(debug=True, port = 7002)
