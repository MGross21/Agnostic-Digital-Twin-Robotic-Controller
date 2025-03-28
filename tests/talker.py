from ..digital_twin import LocalPubSub

talker = LocalPubSub(port=5000)

talker.publish("test", "Hello, World!")