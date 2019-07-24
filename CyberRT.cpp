CyberRT:

1. Talker-Listener
The first part of demonstrating CyberRT API is to understand the Talker/Listener example. 
Following are three essential concepts: node (basic unit), reader(facility to read message) and writer(facility to write message) of the example.

2. Create a node
In the CyberRT framework, the node is the most fundamental unit, similar to the role of a handle. When creating a specific functional object (writer, reader, etc.), 
you need to create it based on an existing node instance. The node creation interface is as follows:

std::unique_ptr<Node> apollo::cyber::CreateNode(const std::string& node_name, const std::string& name_space = "");

Parameters:
node_name: name of the node, globally unique identifier
name_space: name of the space where the node is located
name_space is empty by default. It is the name of the space concatenated with node_name. The format is /namespace/node_name
Return value - An exclusive smart pointer to Node
Error Conditions - when cyber::Init() has not called, the system is in an uninitialized state, unable to create a node, return nullptr

3.Create a writer
The writer is the basic facility used in CyberRT to send messages. Every writer corresponds to a channel with a specific data type. The writer is created by the CreateWriter interface in the node class. The interfaces are listed as below:

template <typename MessageT>
   auto CreateWriter(const std::string& channel_name)
       -> std::shared_ptr<Writer<MessageT>>;
template <typename MessageT>
   auto CreateWriter(const proto::RoleAttributes& role_attr)
       -> std::shared_ptr<Writer<MessageT>>;
Parameters:
channel_name: the name of the channel to write to
MessageT: The type of message to be written out
Return value - Shared pointer to the Writer object

4.Create a reader
The reader is the basic facility used in cyber to receive messages. Reader has to be bound to a callback function when it is created. When a new message arrives in the channel, the callback will be called. The reader is created by the CreateReader interface of the node class. The interfaces are listed as below:

template <typename MessageT>
auto CreateReader(const std::string& channel_name, const std::function<void(const std::shared_ptr<MessageT>&)>& reader_func)
    -> std::shared_ptr<Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const ReaderConfig& config,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
    -> std::shared_ptr<cyber::Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const proto::RoleAttributes& role_attr,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
-> std::shared_ptr<cyber::Reader<MessageT>>;
Parameters:

MessageT: The type of message to read
channel_name: the name of the channel to receive from
reader_func: callback function to process the messages
Return value - Shared pointer to the Reader object

5.Service Creation and Use
	Introduction
In an autonomous driving system, there are many scenarios that require more from module communication than just sending or receiving messages. Service is another way of communication between nodes. Unlike channel, service implements two-way communication, e.g. a node obtains a response by sending a request. This section introduces the service module in CyberRT API with examples.

Demo - Example
Problem: create a client-server model that pass Driver.proto back and forth. When a request is sent in by the client, the server parses/processes the request and returns the response.

The implementation of the demo mainly includes the following steps.













