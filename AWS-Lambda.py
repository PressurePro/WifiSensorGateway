import json
import boto3
import logging

# Set up logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Initialize AWS clients
iot_client = boto3.client('iot')
iot_data_client = boto3.client('iot-data', endpoint_url='https://a1vqmzc3f7mie5-ats.iot.us-west-2.amazonaws.com')

def lambda_handler(event, context):
    logger.info(f"Event received: {json.dumps(event, indent=2)}")
    
    try:
        # The message is directly in the event (not in event['message'] or event['payload'])
        # AWS IoT rule passes the message directly as the event
        message = event
        logger.info(f"Processing message: {json.dumps(message, indent=2)}")
        
        # Check if this is a provisioning request
        if message.get('type') == 'provisioning_request':
            device_id = message.get('device_id')
            logger.info(f"Processing provisioning request for device: {device_id}")
            
            # Generate new certificates for this device
            response = iot_client.create_keys_and_certificate(setAsActive=True)
            certificate_arn = response['certificateArn']
            certificate_id = response['certificateId']
            
            logger.info(f"Generated certificates for device: {device_id}")
            logger.info(f"Certificate ARN: {certificate_arn}")
            logger.info(f"Certificate ID: {certificate_id}")
            
            # Create a policy name for this device
            policy_name = f"DevicePolicy_{device_id}"

            client_id = message.get('client_id', f"esp32c6-{device_id}")  # Use provided client_id or fallback
            
            # Create the policy document for the device
            policy_document = {
                "Version": "2012-10-17",
                "Statement": [
                    {
                        "Effect": "Allow",
                        "Action": [
                            "iot:Connect"
                        ],
                        "Resource": f"arn:aws:iot:us-west-2:*:client/{client_id}"
                    },
                    {
                        "Effect": "Allow",
                        "Action": [
                            "iot:Publish"
                        ],
                        "Resource": [
                            f"arn:aws:iot:us-west-2:*:topic/pressurepro/devices/WifiSensorGateway/{device_id}/sensor-readings",
                            f"arn:aws:iot:us-west-2:*:topic/pressurepro/devices/WifiSensorGateway/{device_id}/diagnostics"
                        ]
                    },
                    {
                        "Effect": "Allow",
                        "Action": [
                            "iot:Subscribe"
                        ],
                        "Resource": [
                            f"arn:aws:iot:us-west-2:*:topicfilter/pressurepro/devices/WifiSensorGateway/{device_id}/commands",
                            f"arn:aws:iot:us-west-2:*:topicfilter/pressurepro/devices/WifiSensorGateway/{device_id}/firmware"
                        ]
                    },
                    {
                        "Effect": "Allow",
                        "Action": [
                            "iot:Receive"
                        ],
                        "Resource": [
                            f"arn:aws:iot:us-west-2:*:topic/pressurepro/devices/WifiSensorGateway/{device_id}/commands",
                            f"arn:aws:iot:us-west-2:*:topic/pressurepro/devices/WifiSensorGateway/{device_id}/firmware"
                        ]
                    }
                ]
            }
            
            try:
                # Create the policy
                iot_client.create_policy(
                    policyName=policy_name,
                    policyDocument=json.dumps(policy_document)
                )
                logger.info(f"Created policy: {policy_name}")
            except iot_client.exceptions.ResourceAlreadyExistsException:
                logger.info(f"Policy {policy_name} already exists, updating...")
                # Update existing policy
                iot_client.create_policy_version(
                    policyName=policy_name,
                    policyDocument=json.dumps(policy_document),
                    setAsDefault=True
                )
            
            # Attach the policy to the certificate
            iot_client.attach_policy(
                policyName=policy_name,
                target=certificate_arn
            )
            logger.info(f"Attached policy {policy_name} to certificate {certificate_id}")
            
            # Create the response payload
            response_payload = {
                'status': 'success',
                'certificates': {
                    'rootca': '''-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----''',
                    'devicecert': response['certificatePem'],
                    'privatekey': response['keyPair']['PrivateKey']
                }
            }
            
            # Publish response to device-specific topic
            response_topic = f"pressurepro/devices/WifiSensorGateway/provisioning/{device_id}/response"
            
            publish_params = {
                'topic': response_topic,
                'payload': json.dumps(response_payload),
                'qos': 1
            }
            
            logger.info(f"Publishing response to topic: {response_topic}")
            iot_data_client.publish(**publish_params)
            
            logger.info("Provisioning response sent successfully")
            
            return {
                'statusCode': 200,
                'body': json.dumps({'message': 'Provisioning completed successfully'})
            }
        else:
            logger.info("Not a provisioning request, ignoring")
            return {
                'statusCode': 200,
                'body': json.dumps({'message': 'Not a provisioning request'})
            }
            
    except KeyError as e:
        logger.error(f"KeyError: {e}")
        return {
            'statusCode': 400,
            'body': json.dumps({'error': f'Missing key: {str(e)}'})
        }
    except json.JSONDecodeError as e:
        logger.error(f"JSON decode error: {e}")
        return {
            'statusCode': 400,
            'body': json.dumps({'error': f'Invalid JSON: {str(e)}'})
        }
    except Exception as e:
        logger.error(f"Error processing message: {e}")
        return {
            'statusCode': 500,
            'body': json.dumps({'error': str(e)})
        }