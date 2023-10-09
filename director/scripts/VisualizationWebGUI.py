import json
from flask import Flask, render_template, request

app = Flask(__name__)

# Initialize a list of objects with their visibility status
objects = {
    "Object 1": True,
    "Object 2": True
    # Add more objects as needed
}

@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        # Update the visibility status based on the checkboxes
        for obj_name in objects:
            objects[obj_name] = bool(request.form.get(obj_name, False))
    return render_template("index.html", objects=objects)


# TODO : The following method doesn't get triggered. Need to sync it with the index function.
@app.route("/read_json", methods=["POST"])
def readJSONFile(self):
    '''
        Function for reading the JSON input file and populating the JSON
        and GUI attributes
    '''   
    
    print("Debugging: Inside readJSONFile function")
    if request.method == "POST":
        # Check if a JSON file was uploaded
        if "json_file" in request.files:
            json_file = request.files["json_file"]

            # Check if the file has a JSON extension
            if json_file.filename.endswith(".json"):
                # Read the JSON data from the uploaded file
                json_data = json.load(json_file)

                # Process the JSON data as needed
                # For example, you can print it or store it in a variable
                print("HEREEE 1!!")
                print(json_data)



    else:
        return jsonify({"error": "Invalid file format. Please upload a JSON file."})

        # Update the visibility status based on the checkboxes
        for obj_name in objects:
            objects[obj_name] = bool(request.form.get(obj_name, False))

    return render_template("index.html", objects=objects)


if __name__ == "__main__":
    # Running the visualization GUI on port 7002.
    app.run(debug=True, port = 7002)
    print("HEREEE!!")