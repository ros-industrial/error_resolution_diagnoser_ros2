export ECS_API=http://127.0.0.1:8000
export AGENT_POST_API=https://postman-echo.com
echo "Checking ECS API Val"
echo $ECS_API


pip3 install -r error_classification_server/requirements.txt
python3 error_classification_server/src/ecs_endpoint.py &
APP_PID=$!
