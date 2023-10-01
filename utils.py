import importlib


def get_topic_class(msg_type):
    try:
        parts = msg_type.split("/")
        package = parts[0]
        message = parts[-1]

        msg_module = importlib.import_module(f"{package}.msg")

        msg_class = getattr(msg_module, message)
        return msg_class
    except (ImportError, AttributeError) as e:
        print(
            f"An error occurred while importing the message type {msg_type}: {str(e)}"
        )
        return None
