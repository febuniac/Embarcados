from flask import Flask, render_template
app = Flask(__name__)

@app.route('/')#this determines the entry point; the / means the root of the website, so just http://127.0.0.1:5000/.
def index():# this is the name we give to the route. Here it was called index because it’s the index of the website.
	return render_template('index.html')#: this is the content of the web page, which is returned when the user browses the index of the website.
@app.route('/cakes')
def cakes():
	return 'Yummy cakes!'
@app.route('/hello/<name>')
def hello(name):
    return render_template('page.html', name=name)


if __name__ == '__main__':
	app.run(debug=True, host='0.0.0.0')#web app will be accessible to any device on the network.
#Open the Pi’s web browser from the taskbar or application menu and navigate to http://127.0.0.1:5000/. You should see a white screen with the words Hello world:






#https://projects.raspberrypi.org/en/projects/python-web-server-with-flask