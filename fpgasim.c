/* Copyright (C) 2022 Connor Claypool
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <vpi_user.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdbool.h>

#define PORT 12346
#define GPIO_EV_QUEUE_SIZE 512

#define NUM_GPIO_INPUT 4
#define NUM_GPIO_OUTPUT 4

#define MSG_LEN 2
#define MSG_PIN_IDX 0
#define MSG_VALUE_IDX 1

#define CLK_PERIOD 0.001
#define HALF_PERIOD (CLK_PERIOD / 2)

#define NETS(cb_data) (nets_t*)((cb_data)->user_data)
#define USER_DATA(data) (PLI_BYTE8*)(data)

// macro for easily defining a GPIO callback function
#define DEFINE_GPIO_OUTPUT_VALUE_CHANGE_CB(__pin) \
PLI_INT32 gpio ## __pin ## _output_change_cb(p_cb_data cb_data) \
{ \
    nets_t *nets = NETS(cb_data); \
    gpio_event_t ev; \
    ev.pin = (__pin); \
    ev.val = get_int_value_from_net(nets->gpio_out[ev.pin]); \
    enqueue_send_event(ev); \
    return 0; \
}

// macro for registering a GPIO callback function
#define REGISTER_GPIO_OUTPUT_VALUE_CHANGE_CB(__pin, __nets) \
    register_value_change_cb(&gpio ## __pin ## _output_change_cb, (__nets)->gpio_out[(__pin)], (__nets))

#define GPIO_IN_NAME(__pin) "gpio_in__pin"

// callback function type
typedef PLI_INT32 (*vpi_cb_t)(struct t_cb_data *);

// struct for top-level I/O of simulated circuit
typedef struct {
    vpiHandle clk;
    vpiHandle gpio_in[NUM_GPIO_INPUT];
    vpiHandle gpio_out[NUM_GPIO_OUTPUT];
    double time_resolution;
} nets_t;

typedef struct {
    uint8_t pin;
    uint8_t val;
} gpio_event_t;

// circular buffer queue
typedef struct {
    // buffer
    gpio_event_t buffer[GPIO_EV_QUEUE_SIZE];
    // current number of items in buffer
    int items;
    // read index
    int read_index;
    // write index
    int write_index;
} gpio_ev_queue_t;

// network communication sockets
int listen_sock, conn_sock;
// threads to handle network communications
pthread_t send_thread;
pthread_t recv_thread;
// queue for sending events
gpio_ev_queue_t send_queue;
// mutex protecting send queue
pthread_mutex_t send_queue_mx;
// queue for receiving events
gpio_ev_queue_t recv_queue;
// mutex protecting recv queue
pthread_mutex_t recv_queue_mx;
// condition variable for signalling sender thread to wake up
pthread_cond_t gpio_event_added;

// check if queue is empty
bool gpio_ev_queue_empty(gpio_ev_queue_t *q)
{
    return q->items == 0;
}

// check if queue is full
bool gpio_ev_queue_full(gpio_ev_queue_t *q)
{
    return q->items == GPIO_EV_QUEUE_SIZE;
}

// pushes item onto queue. Assumes queue is not full!
void gpio_ev_queue_push(gpio_ev_queue_t *q, gpio_event_t ev)
{
    q->buffer[q->write_index] = ev;
    q->write_index += 1;
    if (q->write_index >= GPIO_EV_QUEUE_SIZE) {
        q->write_index = 0;
    }
    q->items += 1;
}

// pops item off queue. Assumes queue is not empty!
gpio_event_t gpio_ev_queue_pop(gpio_ev_queue_t *q)
{
    gpio_event_t ev;

    ev = q->buffer[q->read_index];
    q->read_index += 1;
    if (q->read_index >= GPIO_EV_QUEUE_SIZE) {
        q->read_index = 0;
    }
    q->items -= 1;

    return ev;
}

// enqueue event
void enqueue_send_event(gpio_event_t ev)
{
    pthread_mutex_lock(&send_queue_mx);
    if (!gpio_ev_queue_full(&send_queue)) {
        gpio_ev_queue_push(&send_queue, ev);
        if (send_queue.items == 1) {
            pthread_cond_signal(&gpio_event_added);
        }
    }
    pthread_mutex_unlock(&send_queue_mx);
}

// convert time from double to vpiSimTime
s_vpi_time double_to_vpi_time(double t, double time_resolution)
{
	s_vpi_time vpi_time;
	uint64_t sim_time = (uint64_t)(t * time_resolution);

	vpi_time.type = vpiSimTime;
	vpi_time.low = (uint32_t)(sim_time & __UINT32_MAX__);
	vpi_time.high = (uint32_t)(sim_time >> 32);

	return vpi_time;
}

// regsiter a callback to run after a given delay
void register_delay_cb(vpi_cb_t cb_rtn, double delay, nets_t *nets)
{
    s_vpi_time time = double_to_vpi_time(delay, nets->time_resolution);

    s_cb_data cb = {
        .reason = cbAfterDelay,
        .cb_rtn = cb_rtn,
        .obj = NULL,
        .time = &time,
        .value = NULL,
        .user_data = USER_DATA(nets),
    };

    vpiHandle cb_handle = vpi_register_cb(&cb);
    if (!cb_handle) {
        vpi_printf("Failed to register delay callback!\n");
        exit(1);
    }
    vpi_free_object(cb_handle);
}

// regsiter a callback to run when a net's value changes
void register_value_change_cb(vpi_cb_t cb_rtn, vpiHandle net, nets_t *nets)
{
    s_vpi_time time = {
        .type = vpiSuppressTime,
    };
    s_vpi_value val = {
        .format = vpiSuppressVal,
    };
    s_cb_data cb = {
        .reason = cbValueChange,
        .cb_rtn = cb_rtn,
        .obj = net,
        .time = &time,
        .value = &val,
        .user_data = USER_DATA(nets),
    };

    vpiHandle cb_handle = vpi_register_cb(&cb);
    if (!cb_handle) {
        vpi_printf("Failed to register value change callback!\n");
        exit(1);
    }
    vpi_free_object(cb_handle);
}

// regsiter a callback to run after rw sync
void register_rwsync_cb(vpi_cb_t cb_rtn, nets_t *nets)
{
    s_vpi_time time = {
        .type = vpiSimTime,
        .high = 0,
        .low = 0,
        .real = 0.0,
    };
    s_cb_data cb = {
        .reason = cbReadWriteSynch,
        .cb_rtn = cb_rtn,
        .obj = NULL,
        .time = &time,
        .value = NULL,
        .user_data = USER_DATA(nets),
    };

    vpiHandle cb_handle = vpi_register_cb(&cb);
    if (!cb_handle) {
        vpi_printf("Failed to register value change callback!\n");
        exit(1);
    }
    vpi_free_object(cb_handle);
}

// write an integer value to a net
void write_int_value_to_net(vpiHandle net, int val)
{
    if (net) {
        s_vpi_value value = {
            .format = vpiIntVal,
            .value.integer = val,
        };
        vpi_put_value(net, &value, NULL, vpiNoDelay);
    }
}

// get an integer value from a net
int get_int_value_from_net(vpiHandle net)
{
	int ret = 0; 
    int i;
    int width = vpi_get(vpiSize, net);
	s_vpi_value val = {
        .format = vpiBinStrVal,
    };
	vpi_get_value(net, &val);
	for (i = 0; i < width; i++) {
		ret <<= 1;
		if (val.value.str[i] == '1')
			ret |= 1;
	}

	return ret;
}

// find and save required top level nets
void analyze_toplevel_nets(nets_t *nets)
{
    // VPI object handles to iterator, top level unit and individual net
    vpiHandle iter, top, net;
    // net details: name, width in bits and direction (input/output)
    const char *net_name, *net_dir_str;
    int net_width;
    int net_dir;
    int pin;

    // get a module iterator
    iter = vpi_iterate(vpiModule, NULL);
    // get a handle to the top level unit from the module iterator
    top = vpi_scan(iter);
    // free the iterator object
    vpi_free_object(iter);

    // get the name of the top level unit and print it
    vpi_printf("Top-level unit name: %s\n", vpi_get_str(vpiName, top));

    // get an iterator for the nets (logical wires) within the top level unit
    iter = vpi_iterate(vpiNet, top);
    if (!iter) {
        vpi_free_object(iter);
        return;
    }
    vpi_printf("Iterating through top-level nets...\n");
    // loop through the top-level nets using the iterator
    while ((net = vpi_scan(iter))) {
        net_name = vpi_get_str(vpiName, net);
        net_width = vpi_get(vpiSize, net);
        net_dir = vpi_get(vpiDirection, net);
        switch(net_dir) {
            case vpiInput:
                net_dir_str = "Input";
                break;
            case vpiOutput:
                net_dir_str = "Output";
                break;
            case vpiInout:
                net_dir_str = "Inout";
                break;
            default:
                net_dir_str = "";
                break;
        }
        vpi_printf("Name: %s, Width: %d, Direction: %s\n", net_name, net_width, net_dir_str);
        
        if (strcmp(net_name, "clk") == 0) {
            vpi_printf("Found clk!\n");
            nets->clk = net;
        }
        else if (strlen(net_name) >= 8 && strncmp(net_name, "gpio_in", 7) == 0) {
            pin = atoi(&(net_name[7]));
            if (pin >= 0 && pin < NUM_GPIO_INPUT) {
                vpi_printf("Found GPIO input %d!\n", pin);
                nets->gpio_in[pin] = net;
            } else {
                vpi_free_object(net);
            }
        }
        else if (strlen(net_name) >= 9 && strncmp(net_name, "gpio_out", 8) == 0) {
            pin = atoi(&(net_name[8]));
            if (pin >= 0 && pin < NUM_GPIO_OUTPUT) {
                vpi_printf("Found GPIO output %d!\n", pin);
                nets->gpio_out[pin] = net;
            } else {
                vpi_free_object(net);
            }
        }
        else {
            vpi_free_object(net);
        }
    }
    vpi_free_object(top);
}

DEFINE_GPIO_OUTPUT_VALUE_CHANGE_CB(0);
DEFINE_GPIO_OUTPUT_VALUE_CHANGE_CB(1);
DEFINE_GPIO_OUTPUT_VALUE_CHANGE_CB(2);
DEFINE_GPIO_OUTPUT_VALUE_CHANGE_CB(3);

PLI_INT32 poll_inputs_cb(p_cb_data cb_data);

PLI_INT32 reset_clock_cb(p_cb_data cb_data)
{
    nets_t *nets = NETS(cb_data);
    write_int_value_to_net(nets->clk, 0);
    register_rwsync_cb(&poll_inputs_cb, nets);
    return 0;
}

PLI_INT32 set_clock_cb(p_cb_data cb_data) 
{
    nets_t *nets = NETS(cb_data);
    write_int_value_to_net(nets->clk, 1);
    register_delay_cb(&reset_clock_cb, HALF_PERIOD, nets);
    return 0;
}

// callback once a clock cycle to set simulation inputs
PLI_INT32 poll_inputs_cb(p_cb_data cb_data)
{
    gpio_event_t ev;
    nets_t *nets = NETS(cb_data);

    // pop item off queue
    pthread_mutex_lock(&recv_queue_mx);
    if (!gpio_ev_queue_empty(&recv_queue)) {
        ev = gpio_ev_queue_pop(&recv_queue);
        vpi_printf("Pin: %d, Value: %d\n", ev.pin, ev.val);
        // set simulation input value
        write_int_value_to_net(nets->gpio_in[ev.pin], ev.val);
    }
    pthread_mutex_unlock(&recv_queue_mx);

    // queue clock set callback
    register_delay_cb(&set_clock_cb, HALF_PERIOD, nets);
    return 0;
}

// stage 3 init: reset clock
PLI_INT32 init_stage_3_cb(p_cb_data cb_data)
{
    vpi_printf("Stage 3 init!\n");
    nets_t *nets = NETS(cb_data);
    // reset clock
    write_int_value_to_net(nets->clk, 0);
    // queue input polling callback
    register_delay_cb(&poll_inputs_cb, HALF_PERIOD - 15e-9, nets);
    return 0;
}

// stage 2 init: set clock to 1
PLI_INT32 init_stage_2_cb(p_cb_data cb_data)
{
    vpi_printf("Stage 2 init!\n");
    nets_t *nets = NETS(cb_data);
    // set clock
    write_int_value_to_net(nets->clk, 1);
    // queue stage 3 init
    register_delay_cb(&init_stage_3_cb, 5e-9, nets);
    return 0;
}

// stage 1 init: register value-change callbacks and zero inputs
PLI_INT32 init_stage_1_cb(p_cb_data cb_data)
{
    vpi_printf("Stage 1 init!\n");
    nets_t *nets = NETS(cb_data);

    // register value change callbacks for GPIO outputs
    REGISTER_GPIO_OUTPUT_VALUE_CHANGE_CB(0, nets);
    REGISTER_GPIO_OUTPUT_VALUE_CHANGE_CB(1, nets);
    REGISTER_GPIO_OUTPUT_VALUE_CHANGE_CB(2, nets);
    REGISTER_GPIO_OUTPUT_VALUE_CHANGE_CB(3, nets);

    // zero inputs
    write_int_value_to_net(nets->clk, 0);
    for (int i = 0; i < NUM_GPIO_INPUT; i++) {
        write_int_value_to_net(nets->gpio_in[i], 0);
    }
    // queue stage 2 init
    register_delay_cb(&init_stage_2_cb, 5e-9, nets);
    return 0;
}

// simulation start callback
PLI_INT32 simulation_start_cb(p_cb_data cb_data)
{
    nets_t *nets = NETS(cb_data);

    vpi_printf("Simulation starting...!\n");

    // analyze VHDL circuit and find/save handles to the relevant top-level nets
    analyze_toplevel_nets(nets);

    // queue stage 1 initialization
    register_delay_cb(&init_stage_1_cb, 5e-9, nets);

    return 0;
}

// end of simulation callback
PLI_INT32 simulation_end_cb(p_cb_data cb_data) 
{
    vpi_printf("End of simulation!\n");
    // deallocate nets struct
    nets_t *nets = NETS(cb_data);
    if (nets) {
        free(nets);
        vpi_printf("Nets struct freed!\n");
    }
    return 0;
}

// registers simulation start callback
void register_simulation_start_cb(nets_t *nets)
{
    s_cb_data cb;
    vpiHandle cb_handle;
    cb.reason = cbStartOfSimulation;
    cb.cb_rtn = &simulation_start_cb;
    cb.user_data = USER_DATA(nets);
    cb_handle = vpi_register_cb(&cb);
    if (!cb_handle) {
        vpi_printf("Failed to register simulation start callback!\n");
        exit(1);
    }
    vpi_free_object(cb_handle);
}

// registers simulation end callback
void register_simulation_end_cb(nets_t *nets)
{
    s_cb_data cb;
    vpiHandle cb_handle;
    cb.reason = cbEndOfSimulation;
    cb.cb_rtn = &simulation_end_cb;
    cb.user_data = USER_DATA(nets);
    cb_handle = vpi_register_cb(&cb);
    if (!cb_handle) {
        vpi_printf("Failed to register simulation end callback!\n");
        exit(1);
    }
    vpi_free_object(cb_handle);
}

// init socket and begin listening
int init_socket_and_listen()
{
    int err;
    int listen_sock;
    struct sockaddr_in saddr;

    // initialize socket
    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    // allow address/port reuse without timeout
    err = setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &(int){1}, sizeof(int));
    if (err) {
        vpi_printf("Error setting socket option: %d\n", err);
    }

    // initialize listen address
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(PORT);

    // bind socket to address & port
    err = bind(listen_sock, (struct sockaddr *)&saddr, sizeof(saddr));
	if (err) {
		vpi_printf("Error binding socket: %d\n", err);
	}

    // listen for incoming connections
	err = listen(listen_sock, 2);
	if (err) {
		vpi_printf("Error listening\n");
	}

    return listen_sock;
}

// receive message
int receive_message(int sock, uint8_t *buf, int len)
{
	int ret;
	int bytes_read = 0;

    // check bounds
    if (len < MSG_LEN) {
        return -1;
    }

	do
	{
        // receive message from socket
		ret = read(sock, buf + bytes_read, MSG_LEN - bytes_read);
		if (ret < 0) {
			vpi_printf("Error receiving on socket: %d\n", ret);
		} else {
			bytes_read += ret;
		}
    // repeat if full message was not received
	} while (bytes_read < MSG_LEN);

    return bytes_read;
}

// recv thread function
void *recv_thread_fn(void *unused)
{
    uint8_t msg_buf[MSG_LEN];
    gpio_event_t ev;

    vpi_printf("Recv thread created!\n");
    while (1) {
        // receive message
        receive_message(conn_sock, msg_buf, MSG_LEN);
        // extract values
        ev.pin = msg_buf[MSG_PIN_IDX];
        ev.val = msg_buf[MSG_VALUE_IDX];
        // add to queue if it is not full
        pthread_mutex_lock(&recv_queue_mx);
        if (!gpio_ev_queue_full(&recv_queue)) {
            gpio_ev_queue_push(&recv_queue, ev);
        } else {
            vpi_printf("Recv queue full!\n");
        }
        pthread_mutex_unlock(&recv_queue_mx);

    }

    return NULL;
}

// format message
int generate_msg(gpio_event_t ev, uint8_t *msg, int len)
{
    // check bounds
    if (len < MSG_LEN) {
        return -1;
    }
	// place message into buffer
	msg[MSG_PIN_IDX] = ev.pin;
	msg[MSG_VALUE_IDX] = ev.val;
    return 0;
}

// send message
int send_message(uint8_t *msg, int len)
{
    int ret;
    int bytes_written = 0;

    // check bounds
    if (len < MSG_LEN) {
        return -1;
    }

    do
    {
        // send message through socket
        ret = write(conn_sock, msg + bytes_written, MSG_LEN - bytes_written);
        if (ret < 0) {
            vpi_printf("Error sending message: %d\n", ret);
        } else {
            bytes_written += ret;
        }
    // retry if whole message was not sent
    } while (bytes_written < MSG_LEN);
     
    return bytes_written;
}


// send thread function
void *send_thread_fn(void *unused)
{
    gpio_event_t ev;
    bool got_event;
    uint8_t msg_buf[MSG_LEN];

    vpi_printf("Send thread created!\n");

    while (1) {
        // check send queue
        pthread_mutex_lock(&send_queue_mx);
        if (!gpio_ev_queue_empty(&send_queue)) {
            ev = gpio_ev_queue_pop(&send_queue);
            got_event = true;
        } else {
            got_event = false;
        }
        pthread_mutex_unlock(&send_queue_mx);

        if (got_event) {
            generate_msg(ev, msg_buf, MSG_LEN);
            vpi_printf("Send: pin: %d, value: %d\n", ev.pin, ev.val);
            send_message(msg_buf, MSG_LEN);
        } else {
            // wait until signalled and then re-init condvar
            pthread_mutex_lock(&send_queue_mx);
            pthread_cond_wait(&gpio_event_added, &send_queue_mx);
            pthread_cond_destroy(&gpio_event_added);
            pthread_cond_init(&gpio_event_added, NULL);
            pthread_mutex_unlock(&send_queue_mx);
        }
    }

    return NULL;
}

// entry point callback
void entry_point_cb()
{
    int err;
    nets_t *nets;

    // init sync objects
    if (pthread_mutex_init(&send_queue_mx, NULL)) {
        vpi_printf("Init send mutex failed!\n");
    }
    if (pthread_mutex_init(&recv_queue_mx, NULL)) {
        vpi_printf("Init recv mutex failed!\n");
    }
    if (pthread_cond_init(&gpio_event_added, NULL)) {
        vpi_printf("Init cond var failed!\n");
    }

    // listen for incoming connections
    listen_sock = init_socket_and_listen();
    // wait for successful connection
    conn_sock = accept(listen_sock, (struct sockaddr *)NULL, NULL);

    // launch threads to handle sending and receiving messages
    err = pthread_create(&recv_thread, NULL, recv_thread_fn, NULL);
    if (err) {
        vpi_printf("Error creating recv thread: %d\n", err);
    }
    err = pthread_create(&send_thread, NULL, send_thread_fn, NULL);
    if (err) {
        vpi_printf("Error creating send thread: %d\n", err);
    }

    // allocate memory for nets struct
    nets = (nets_t*)malloc(sizeof(nets_t));
    // set time resolution
    nets->time_resolution = pow(10, -vpi_get(vpiTimePrecision, NULL));
    // register simulation start and end callbacks
    register_simulation_start_cb(nets);
    register_simulation_end_cb(nets);
}

// define entry point callback
void (*vlog_startup_routines[]) () = {entry_point_cb, 0};