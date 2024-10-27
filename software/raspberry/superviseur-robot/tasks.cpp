/*
 * Copyright (C) 2018 dimercur
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_RELOADWD 25 // TODO: ADJUST THIS PRIORITY

/*                            --s-ms-us-ns */
RTIME reloadWD_period_ns =      1000000000llu ;
RTIME updateBattery_period_ns =  500000000llu ;
RTIME captureImage_period_ns =   100000000llu ;

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_battery, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_counter, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_capture_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_accept_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_compute, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_server, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_watchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_camera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_restart_server, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_RELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_updateBattery, "th_updateBattery", 0, PRIORITY_RELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_RELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImage, "th_sendImage", 0, PRIORITY_RELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks configuration                                                                */
    /**************************************************************************************/

    if (err = rt_task_set_periodic(&th_reloadWD, TM_NOW, rt_timer_ns2ticks(reloadWD_period_ns))) {
        cerr << "Error task set periodic: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_set_periodic(&th_updateBattery, TM_NOW, rt_timer_ns2ticks(updateBattery_period_ns))) {
        cerr << "Error task set periodic: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_set_periodic(&th_sendImage, TM_NOW, rt_timer_ns2ticks(captureImage_period_ns))) {
        cerr << "Error task set periodic: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_updateBattery, (void(*)(void*)) & Tasks::updateBattery, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::openCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImage, (void(*)(void*)) & Tasks::sendImage, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    monitor.Close();
    rt_mutex_release(&mutex_monitor);
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();
    rt_mutex_release(&mutex_robot);
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    while(1){
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };

        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        rt_mutex_acquire(&mutex_server, TM_INFINITE);
        server_open = true;
        rt_mutex_release(&mutex_server);
        rt_sem_broadcast(&sem_serverOk);
        
        rt_sem_p(&sem_restart_server, TM_INFINITE);
    }
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    bool server_up = false;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/

    while(1){
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        rt_mutex_acquire(&mutex_server, TM_INFINITE);
            server_up = server_open;
        rt_mutex_release(&mutex_server);
        while (server_up) {
            rt_mutex_acquire(&mutex_server, TM_INFINITE);
            server_up = server_open;
            rt_mutex_release(&mutex_server);
            if(server_up){
                cout << "wait msg to send" << endl << flush;
                msg = ReadInQueue(&q_messageToMon);
                cout << "Send msg to mon: " << msg->ToString() << endl << flush;
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msg); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
            }else{

            }
        }
            
        }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    bool server_up = false;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    //rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        rt_mutex_acquire(&mutex_server, TM_INFINITE);
            server_up = server_open;
        rt_mutex_release(&mutex_server);
        while (server_up) {
            msgRcv = monitor.Read();
            cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

            if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
                cout << "Connection lost between Supervisor and Monitor" << endl << flush; // Fonctionnalite 5

                rt_mutex_acquire(&mutex_server, TM_INFINITE);
                server_open = false;
                rt_mutex_release(&mutex_server);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Close();
                rt_mutex_release(&mutex_monitor);
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close(); // close_communication_robot
                rt_mutex_release(&mutex_robot);
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0; // Reset -> to initial state
                rt_mutex_release(&mutex_robotStarted);
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                camera->Close();
                rt_mutex_release(&mutex_camera);

                rt_mutex_acquire(&mutex_counter, TM_INFINITE);
                lost_messages_counter = 0;
                rt_mutex_release(&mutex_counter);

                rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
                watchdog = false;
                rt_mutex_release(&mutex_watchdog);

                rt_mutex_acquire(&mutex_battery, TM_INFINITE);
                battery = false;
                rt_mutex_release(&mutex_battery);
                
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                open_camera = false;
                rt_mutex_release(&mutex_camera);
                
                rt_mutex_acquire(&mutex_capture_arena, TM_INFINITE);
                capture_arena = false;
                rt_mutex_release(&mutex_capture_arena);
               
                rt_mutex_acquire(&mutex_accept_arena, TM_INFINITE);
                accept_arena = false;
                rt_mutex_release(&mutex_accept_arena);

                rt_mutex_acquire(&mutex_compute, TM_INFINITE);
                compute = false;
                rt_mutex_release(&mutex_compute);

                rt_mutex_acquire(&mutex_server, TM_INFINITE);
                server_open = false;
                rt_mutex_release(&mutex_server);


                
                // TODO: finish Fonc 6
                rt_sem_v(&sem_restart_server);
                //delete(msgRcv);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_v(&sem_openComRobot); // Libère le (sémaphore de) démarrage du robot
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
                rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
                watchdog = false;
                rt_mutex_release(&mutex_watchdog);
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
                rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
                watchdog = true;
                rt_mutex_release(&mutex_watchdog);
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)) {
                rt_mutex_acquire(&mutex_battery, TM_INFINITE);
                battery = true;
                rt_mutex_release(&mutex_battery);
            } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                open_camera = true;
                rt_mutex_release(&mutex_camera);
                rt_sem_v(&sem_camera);
            } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                open_camera = false;
                rt_mutex_release(&mutex_camera);
                rt_sem_v(&sem_camera);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
                rt_mutex_acquire(&mutex_capture_arena, TM_INFINITE);
                capture_arena = true;
                rt_mutex_release(&mutex_capture_arena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
                rt_mutex_acquire(&mutex_accept_arena, TM_INFINITE);
                accept_arena = true;
                rt_mutex_release(&mutex_accept_arena);
                rt_sem_v(&sem_arena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
                rt_mutex_acquire(&mutex_accept_arena, TM_INFINITE);
                accept_arena = false;
                rt_mutex_release(&mutex_accept_arena);
                rt_sem_v(&sem_arena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
                rt_mutex_acquire(&mutex_compute, TM_INFINITE);
                compute = true;
                rt_mutex_release(&mutex_compute);
            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
                rt_mutex_acquire(&mutex_compute, TM_INFINITE);
                compute = false;
                rt_mutex_release(&mutex_compute);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            }
            delete(msgRcv); // must be deleted manually, no consumer
        }
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK); // Message d'erreur
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    bool wd;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);

        rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
        wd = watchdog;
        rt_mutex_release(&mutex_watchdog);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);

        if(wd){
            msgSend = robot.Write(robot.StartWithWD());
            //rt_sem_v(&sem_watchdog); // synchronize with reload of WD
            cout << "Start Robot with watchdog (";
        }else{
            msgSend = robot.Write(robot.StartWithoutWD());
            cout << "Start Robot without watchdog (";
        }
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            monitor.Write(new Message(MESSAGE_ANSWER_ACK));
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        } else {
            monitor.Write(new Message(MESSAGE_ANSWER_NACK));
            cout << "Robot not started" << endl << flush;
        }
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    int counter;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            if((robot.Write(new Message((MessageID)cpMove))->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))){
                rt_mutex_acquire(&mutex_counter, TM_INFINITE);
                lost_messages_counter++;
                rt_mutex_release(&mutex_counter);
            }else{
                rt_mutex_acquire(&mutex_counter, TM_INFINITE);
                lost_messages_counter = 0;
                rt_mutex_release(&mutex_counter);
            }
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_counter, TM_INFINITE);
            counter = lost_messages_counter;
            rt_mutex_release(&mutex_counter);
            if(counter >= 3){
                cout << "Communication lost between Supervisor and Robot " << endl << flush;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close(); // close_communication_robot
                rt_mutex_release(&mutex_robot);
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0; // Reset -> to initial state
                rt_mutex_release(&mutex_robotStarted);
            }
        }
        cout << endl << flush;
    }
}


/**
* @brief Thread handling reload of the watchdog.
*/
void Tasks::ReloadWD(void *arg){
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    //rt_sem_p(&sem_watchdog, TM_INFINITE); // synchronize with start of WD

    while(1){
        cout << "Periodic watchdog reload" << endl << flush;
        err = rt_task_wait_period(NULL);
        if (err) {
            printf ("error on wait_period, %s \n ", strerror(-err));
            break ;
        }
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
        rt_mutex_release(&mutex_robot);
    }
}

/**
* @brief Thread updating the battery level.
*/
void Tasks::updateBattery(void *arg){
    int err;
    int counter;
    BatteryLevel batteryLvl;
    bool getBattery = false;
    MessageBattery *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while(1){
        cout << "Periodic battery update" << endl << flush;
        err = rt_task_wait_period(NULL) ;
        if (err) {
            printf ("error on wait_period, %s \n ", strerror(-err));
            break ;
        }
        rt_mutex_acquire(&mutex_battery, TM_INFINITE);
        getBattery = battery;
        rt_mutex_release(&mutex_battery);
        
        if(getBattery){
            //cout << "battery lvl asked" << endl << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); 
            msg = (MessageBattery*) robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));


            if((msg->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))){
                rt_mutex_acquire(&mutex_counter, TM_INFINITE);
                lost_messages_counter++;
                rt_mutex_release(&mutex_counter);
            }else{
                rt_mutex_acquire(&mutex_counter, TM_INFINITE);
                lost_messages_counter = 0;
                rt_mutex_release(&mutex_counter);
            }
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_counter, TM_INFINITE);
            counter = lost_messages_counter;
            rt_mutex_release(&mutex_counter);
            if(counter >= 3){
                cout << "Communication lost between Supervisor and Robot " << endl << flush;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Close(); // close_communication_robot
                rt_mutex_release(&mutex_robot);
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0; // Reset -> to initial state
                rt_mutex_release(&mutex_robotStarted);
            }


            batteryLvl = msg->GetLevel();
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new MessageBattery(MESSAGE_ROBOT_BATTERY_LEVEL, batteryLvl));
            rt_mutex_release(&mutex_monitor);
        }
    }
}

/**
* @brief Thread opening the camera.
*/
void Tasks::openCamera(void *arg){

    bool openCam = false;
    bool is_open = false;
    Message * msgSend;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    

    while(1){
        rt_sem_p(&sem_camera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        openCam = open_camera;
        is_open = camera->IsOpen();

        if(openCam && !is_open){
            if(!camera->Open()){ // Opening camera Failed
                cout << "Failed to open camera" << endl << flush;
            }else{
                cout << "Successfully opened Camera" << endl << flush;
            }
        }else if(!openCam && is_open){
            camera->Close();
            cout << "Camera succesfully closed" << endl << flush;
        }
        rt_mutex_release(&mutex_camera);  
    }
}

/**
* @brief Thread sending an image to the monitor.
*/
void Tasks::sendImage(void *arg){
    Img* image;
    Arena* arena_loc;
    int err;
    bool cam_open = false;
    bool capt_arena = false;
    bool valid_arena = false;
    bool accept_arena_loc = false;
    bool compute_robot = false;


    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while(1){
        err = rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        cam_open = camera->IsOpen();
        rt_mutex_release(&mutex_camera);
        rt_mutex_acquire(&mutex_capture_arena, TM_INFINITE);
        capt_arena = capture_arena;
        rt_mutex_release(&mutex_capture_arena);

        if(cam_open){
            cout << "Periodic image sending" << endl << flush;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            image = new Img(camera->Grab());
            rt_mutex_release(&mutex_camera);
            rt_mutex_acquire(&mutex_compute, TM_INFINITE);
            compute_robot = compute;
            rt_mutex_release(&mutex_compute);
            rt_mutex_acquire(&mutex_accept_arena, TM_INFINITE);
            accept_arena_loc = accept_arena;
            rt_mutex_release(&mutex_accept_arena);


            /** Gestion image Arène */
            if(capt_arena){
                arena_loc = new Arena(image->SearchArena());
                if(arena_loc->IsEmpty()){
                    cout << "[INFO] No Arena found" << endl << flush;
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new Message(MESSAGE_ANSWER_NACK));
                    rt_mutex_release(&mutex_monitor);
                } else {
                    cout << "[INFO] Arena found" << endl << flush;
                    image->DrawArena(*arena_loc);
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new MessageImg(MESSAGE_CAM_IMAGE, image));
                    rt_mutex_release(&mutex_monitor);
                    rt_sem_p(&sem_arena, TM_INFINITE);
                    rt_mutex_acquire(&mutex_accept_arena, TM_INFINITE);
                    valid_arena = accept_arena;
                    rt_mutex_release(&mutex_accept_arena);

                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    if(!valid_arena){
                        arena = NULL;
                    }else{
                        arena = arena_loc;
                    }
                    rt_mutex_release(&mutex_arena);

                }
                rt_mutex_acquire(&mutex_capture_arena, TM_INFINITE);
                capture_arena = false;
                rt_mutex_release(&mutex_capture_arena);
            } else if(accept_arena_loc){
                    image->DrawArena(*arena_loc);
            } else if(!accept_arena_loc){
                arena = NULL;
            }
            /** Gestion image Robot */
            if(compute_robot && arena != NULL){
                std::list<Position> position_list = image->SearchRobot(*arena);
                Position *position = new Position();
                if(!position_list.empty()){
                    *position = position_list.front();
                    image->DrawRobot(*position);
                }
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new MessagePosition(MESSAGE_CAM_POSITION, *position));
                    rt_mutex_release(&mutex_monitor);
            }

            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new MessageImg(MESSAGE_CAM_IMAGE, image));
            rt_mutex_release(&mutex_monitor);

        } 
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}