from flask import Flask, request, render_template	
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

todos = {}

class HelloWorld(Resource):
    def get(self):
        return {'hello': 'world'}

class Rangeset1(Resource):
    def get(self):
        return {'Temp1$': '23','Temp2': '24','Temp3': '25'}


class Rangeset2(Resource):
    def get(self):
    	return request.args.get('form_name')

class submit(Resource):
	app.route('/hi',methods=['GET','POST'])
	def print_form():
	    if request.method == 'POST':
	        return render_template('form.html',result=request.form['fooput'])
	    if request.method == 'GET':
	        return render_template('form.html')	

@app.route('/login', methods=['POST', 'GET'])
def login():
    error = None
    if request.method == 'POST':
        if valid_login(request.form['username'],
                       request.form['password']):
            return log_the_user_in(request.form['username'])
        else:
            error = 'Invalid username/password'
    # the code below is executed if the request method
    # was GET or the credentials were invalid
    return render_template('login.html', error=error)

# @app.route('/quiz')
# def quiz():
#     return render_template('quiz.html')

# @app.route('/quiz_answers', methods=['POST'])
# def quiz_answers():
#     q1 = request.form['q1']
#     q2 = request.form['q2']
#     q4 = request.form['q4']
#     q5 = request.form['q5']        	


#class TodoSimple(Resource):
#    def get(self, todo_id):
#        return {todo_id: todos[todo_id]}		
#
#    def put(self, todo_id):
#        todos[todo_id] = request.form['data']
#        return {todo_id: todos[todo_id]}

#api.add_resource(TodoSimple, '/<string:todo_id>')
api.add_resource(HelloWorld, '/')
api.add_resource(Rangeset1, '/set1')
api.add_resource(Rangeset2, '/set2')
api.add_resource(submit, '/hi')
#api.add_resource(quiz, '/quiz')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
    #app.run(debug=True)