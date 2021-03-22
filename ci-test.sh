export ECS_API=http://127.0.0.1:8000
export AGENT_POST_API=https://postman-echo.com
echo "Checking ECS API Val"
echo $ECS_API

pip3 install wheel
pip3 install sanic==20.12.0
pip3 install databases[sqlite]==0.2.6

pip3 uninstall -y sqlalchemy
pip3 install sqlalchemy==1.3.13

#pip3 install -r error_classification_server/requirements.txt
python3 error_classification_server/src/ecs_endpoint.py &
APP_PID=$!
