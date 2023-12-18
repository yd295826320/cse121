from flask import Flask, request
app = Flask(__name__)

@app.route('/readfile/location.txt', methods=['GET'])
def read_location():
    try:
        with open('location.txt', 'r') as file:
            location = file.read()
        return location
    except FileNotFoundError:
        return "The file location.txt does not exist."

@app.route('/weather', methods=['POST'])
def post_weather():
    content = request.data.decode('utf-8')
    #with open('weather.txt', 'a') as file:
    #    file.write(content + '\n')
    #return content
    print(f"Post, Content:{content}")
    return "POST request received"

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=1234)

