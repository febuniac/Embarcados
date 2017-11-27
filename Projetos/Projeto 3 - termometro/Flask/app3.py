from flask import Flask, request, render_template
from flask_restful import Resource, Api
import csv
import os
app = Flask(__name__)
api = Api(app)

@app.route("/", methods=['GET', 'POST'])
def index():
    return render_template("forms.html")

@app.route("/submittemp", methods=['GET', 'POST'])
def submittemp():
    # methods is an array that's used in Flask which requests' methods are
# allowed to be performed in this route.
    # This is to make sure the HTTP method is POST and not any other
    if request.method == 'POST':
        # request.form is a dictionary that contains the form sent through
        # the HTTP request. This work by getting the name="xxx" attribute of
        # the html form field. So, if you want to get the name, your input
        # should be something like this: <input type="text" name="name" />.
        green = request.form['greent']
        yellow = request.form['yellowt']
        red = request.form['redt']

        # This array is the fields your csv file has and in the following code
        # you'll see how it will be used. Change it to your actual csv's fields.
        fieldtemp = ['greent', 'yellowt','redt']

        # We repeat the same step as the reading, but with "w" to indicate
        # the file is going to be written.
        with open('tempList.csv','w') as inFile:
            # DictWriter will help you write the file easily by treating the
            # csv as a python's class and will allow you to work with
            # dictionaries instead of having to add the csv manually.
            writer = csv.DictWriter(inFile, fieldnames=fieldtemp)

            # writerow() will write a row in your csv file
            writer.writerow({'greent': green, 'yellowt': yellow, 'redt':red})

        # And you return a text or a template, but if you don't return anything
        # this code will never work.
    return render_template("done.html")
#fun var

filepath = os.path.join(os.path.dirname(__file__),'tempList.csv')

open_read = open(filepath,'r')
page =''

while True:
	read_data = open_read.readline()
	page += '<p>%s</p>' % read_data
	mylist = page.split(',')
	if open_read.readline() == '':
		break
@app.route("/temp1")
def temp1():
	return mylist[0] 
@app.route("/temp2")
def temp2():
	return mylist[1] 
@app.route("/temp3")
def temp3():
	return mylist[2] 
@app.route("/temperatures")
def temperatures():
	return page  
	#api.add_resource(index, '/')
if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)  
    #app.run(debug=True)