from flask import Flask, request, render_template
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

@app.route("/", methods=['GET', 'POST'])
def index():
    return render_template("forms.html")

@app.route("/submittemp", methods=['GET', 'POST'])
def submittemp():
    #x = (request.form['projectFilepath'])
    return render_template("done.html")
    
#api.add_resource(index, '/')
if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)  
    #app.run(debug=True)