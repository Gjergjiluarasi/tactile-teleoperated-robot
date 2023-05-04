#include <cpprest/http_listener.h>
#include <cpprest/json.h>

using namespace web;
using namespace http;
using namespace http::experimental::listener;

// Define the API endpoint for receiving the button state
void handle_button_state(const http_request &request)
{
    // Read the JSON payload from the request body
    request.extract_json().then([](json::value payload)
                                {
        // Extract the button state from the JSON payload
        bool button_state = payload["button_state"].as_bool();

        // Update the actuator value based on the button state
        // ...

        // Return a response to the frontend
        http_response response(status_codes::OK);
        response.headers().add("Access-Control-Allow-Origin", "*");
        response.headers().add("Content-Type", "application/json");
        response.set_body("{\"success\": true}");
        request.reply(response); });
}

int main()
{
    // Create an HTTP listener on port 8080
    http_listener listener("http://localhost:8080");
    listener.support(methods::POST, handle_button_state);
    listener.open().wait();

    // Keep the listener running
    while (true)
        ;
    return 0;
}
