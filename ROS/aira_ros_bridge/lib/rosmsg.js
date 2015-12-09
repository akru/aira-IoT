var message_path = './msg/';

module.exports = {
    load: function (msg_type) {
        return require(message_path + msg_type);
    }
}
