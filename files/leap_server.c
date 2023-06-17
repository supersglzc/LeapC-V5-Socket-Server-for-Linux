/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <time.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "LeapC.h"
#include "ExampleConnection.h"

#define PORT 8080

int64_t lastFrameID = 0; //The last frame received

int main(int argc, char** argv) {
  // ------------------------------
  // Leap setup
  // ------------------------------
  printf("LeapSocketServer V1.0\n");
  OpenConnection();
  while(!IsConnected)
    millisleep(100); //wait a bit to let the connection complete

  printf("Connected Leap.");
  LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
  if(deviceProps)
    printf("Using device %s.\n", deviceProps->serial);

  // ------------------------------
  // Socket setup
  // ------------------------------

  int sockfd, newsockfd;
  socklen_t clilen;
  bool status = false;
  char buffer[4096];
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  printf("Creating socket.");
  // create socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
      perror("ERROR opening socket");
      exit(1);
  }

  printf("Binding socket.\n");
  // bind socket to address
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(PORT);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
      perror("ERROR on binding");
      exit(1);
  }

  printf("Listening for incoming socket connections.");
  // listen for incoming connections
  listen(sockfd, 5);
  clilen = sizeof(cli_addr);

  //printf("Accepting.\n");
  // accept incoming connection
  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
  if (newsockfd < 0) {
      perror("ERROR on accept");
      exit(1);
  }

  printf("Start sending frames.");
  for(;;){
    LEAP_TRACKING_EVENT *frame = GetFrame();
    if(frame && (frame->tracking_frame_id > lastFrameID)){
      lastFrameID = frame->tracking_frame_id;
      //printf("Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);
      
      float data_send[213];  // 209
      if(frame->nHands<1){

        //invalid data because not enough hands recognized
        float data[] = {
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0
        };
        memcpy(data_send, data, sizeof(data));
        status = false;
      }else{
        //for(uint32_t h = 0; h < frame->nHands; h++){
        LEAP_HAND* hand = &frame->pHands[0];
        
        // Valid connection
        float data[] = {
	  // palm
	  hand->palm.direction.x,
	  hand->palm.direction.y,
	  hand->palm.direction.z,
	  hand->palm.normal.x,
	  hand->palm.normal.y,
	  hand->palm.normal.z,
          hand->palm.position.x,
          hand->palm.position.y,
          hand->palm.position.z,
	  hand->palm.orientation.x,
          hand->palm.orientation.y,
          hand->palm.orientation.z,
	  hand->palm.orientation.w,
	  // thumb
	  hand->thumb.distal.rotation.x,
          hand->thumb.distal.rotation.y,
          hand->thumb.distal.rotation.z,
	  hand->thumb.distal.rotation.w,
	  hand->thumb.metacarpal.rotation.x,
          hand->thumb.metacarpal.rotation.y,
          hand->thumb.metacarpal.rotation.z,
          hand->thumb.metacarpal.rotation.w,
	  hand->thumb.proximal.rotation.x,
          hand->thumb.proximal.rotation.y,
          hand->thumb.proximal.rotation.z,
          hand->thumb.proximal.rotation.w,
	  hand->thumb.intermediate.rotation.x,
          hand->thumb.intermediate.rotation.y,
          hand->thumb.intermediate.rotation.z,
          hand->thumb.intermediate.rotation.w,
	  hand->thumb.distal.next_joint.x,
          hand->thumb.distal.next_joint.y,
          hand->thumb.distal.next_joint.z,
          hand->thumb.metacarpal.next_joint.x,
          hand->thumb.metacarpal.next_joint.y,
          hand->thumb.metacarpal.next_joint.z,
          hand->thumb.proximal.next_joint.x,
          hand->thumb.proximal.next_joint.y,
          hand->thumb.proximal.next_joint.z,
          hand->thumb.intermediate.next_joint.x,
          hand->thumb.intermediate.next_joint.y,
          hand->thumb.intermediate.next_joint.z,
          hand->thumb.distal.prev_joint.x,
          hand->thumb.distal.prev_joint.y,
          hand->thumb.distal.prev_joint.z,
          hand->thumb.metacarpal.prev_joint.x,
          hand->thumb.metacarpal.prev_joint.y,
          hand->thumb.metacarpal.prev_joint.z,
          hand->thumb.proximal.prev_joint.x,
          hand->thumb.proximal.prev_joint.y,
          hand->thumb.proximal.prev_joint.z,
          hand->thumb.intermediate.prev_joint.x,
          hand->thumb.intermediate.prev_joint.y,
          hand->thumb.intermediate.prev_joint.z,
	  // index
	  hand->index.distal.rotation.x,
          hand->index.distal.rotation.y,
          hand->index.distal.rotation.z,
          hand->index.distal.rotation.w,
          hand->index.metacarpal.rotation.x,
          hand->index.metacarpal.rotation.y,
          hand->index.metacarpal.rotation.z,
          hand->index.metacarpal.rotation.w,
          hand->index.proximal.rotation.x,
          hand->index.proximal.rotation.y,
          hand->index.proximal.rotation.z,
          hand->index.proximal.rotation.w,
          hand->index.intermediate.rotation.x,
          hand->index.intermediate.rotation.y,
          hand->index.intermediate.rotation.z,
          hand->index.intermediate.rotation.w,
	  hand->index.distal.next_joint.x,
          hand->index.distal.next_joint.y,
          hand->index.distal.next_joint.z,
          hand->index.metacarpal.next_joint.x,
          hand->index.metacarpal.next_joint.y,
          hand->index.metacarpal.next_joint.z,
          hand->index.proximal.next_joint.x,
          hand->index.proximal.next_joint.y,
          hand->index.proximal.next_joint.z,
          hand->index.intermediate.next_joint.x,
          hand->index.intermediate.next_joint.y,
          hand->index.intermediate.next_joint.z,
          hand->index.distal.prev_joint.x,
          hand->index.distal.prev_joint.y,
          hand->index.distal.prev_joint.z,
          hand->index.metacarpal.prev_joint.x,
          hand->index.metacarpal.prev_joint.y,
          hand->index.metacarpal.prev_joint.z,
          hand->index.proximal.prev_joint.x,
          hand->index.proximal.prev_joint.y,
          hand->index.proximal.prev_joint.z,
          hand->index.intermediate.prev_joint.x,
          hand->index.intermediate.prev_joint.y,
          hand->index.intermediate.prev_joint.z,
	  // middle
	  hand->middle.distal.rotation.x,
          hand->middle.distal.rotation.y,
          hand->middle.distal.rotation.z,
          hand->middle.distal.rotation.w,
          hand->middle.metacarpal.rotation.x,
          hand->middle.metacarpal.rotation.y,
          hand->middle.metacarpal.rotation.z,
          hand->middle.metacarpal.rotation.w,
          hand->middle.proximal.rotation.x,
          hand->middle.proximal.rotation.y,
          hand->middle.proximal.rotation.z,
          hand->middle.proximal.rotation.w,
          hand->middle.intermediate.rotation.x,
          hand->middle.intermediate.rotation.y,
          hand->middle.intermediate.rotation.z,
          hand->middle.intermediate.rotation.w,
      	  hand->middle.distal.next_joint.x,
          hand->middle.distal.next_joint.y,
          hand->middle.distal.next_joint.z,
          hand->middle.metacarpal.next_joint.x,
          hand->middle.metacarpal.next_joint.y,
          hand->middle.metacarpal.next_joint.z,
          hand->middle.proximal.next_joint.x,
          hand->middle.proximal.next_joint.y,
          hand->middle.proximal.next_joint.z,
          hand->middle.intermediate.next_joint.x,
          hand->middle.intermediate.next_joint.y,
          hand->middle.intermediate.next_joint.z,
          hand->middle.distal.prev_joint.x,
          hand->middle.distal.prev_joint.y,
          hand->middle.distal.prev_joint.z,
          hand->middle.metacarpal.prev_joint.x,
          hand->middle.metacarpal.prev_joint.y,
          hand->middle.metacarpal.prev_joint.z,
          hand->middle.proximal.prev_joint.x,
          hand->middle.proximal.prev_joint.y,
          hand->middle.proximal.prev_joint.z,
          hand->middle.intermediate.prev_joint.x,
          hand->middle.intermediate.prev_joint.y,
          hand->middle.intermediate.prev_joint.z,	  
	  // ring
	  hand->ring.distal.rotation.x,
          hand->ring.distal.rotation.y,
          hand->ring.distal.rotation.z,
          hand->ring.distal.rotation.w,
          hand->ring.metacarpal.rotation.x,
          hand->ring.metacarpal.rotation.y,
          hand->ring.metacarpal.rotation.z,
          hand->ring.metacarpal.rotation.w,
          hand->ring.proximal.rotation.x,
          hand->ring.proximal.rotation.y,
          hand->ring.proximal.rotation.z,
          hand->ring.proximal.rotation.w,
          hand->ring.intermediate.rotation.x,
          hand->ring.intermediate.rotation.y,
          hand->ring.intermediate.rotation.z,
          hand->ring.intermediate.rotation.w,
	  hand->ring.distal.next_joint.x,
          hand->ring.distal.next_joint.y,
          hand->ring.distal.next_joint.z,
          hand->ring.metacarpal.next_joint.x,
          hand->ring.metacarpal.next_joint.y,
          hand->ring.metacarpal.next_joint.z,
          hand->ring.proximal.next_joint.x,
          hand->ring.proximal.next_joint.y,
          hand->ring.proximal.next_joint.z,
          hand->ring.intermediate.next_joint.x,
          hand->ring.intermediate.next_joint.y,
          hand->ring.intermediate.next_joint.z,
          hand->ring.distal.prev_joint.x,
          hand->ring.distal.prev_joint.y,
          hand->ring.distal.prev_joint.z,
          hand->ring.metacarpal.prev_joint.x,
          hand->ring.metacarpal.prev_joint.y,
          hand->ring.metacarpal.prev_joint.z,
          hand->ring.proximal.prev_joint.x,
          hand->ring.proximal.prev_joint.y,
          hand->ring.proximal.prev_joint.z,
          hand->ring.intermediate.prev_joint.x,
          hand->ring.intermediate.prev_joint.y,
          hand->ring.intermediate.prev_joint.z,
	  // pinky
	  hand->pinky.distal.rotation.x,
          hand->pinky.distal.rotation.y,
          hand->pinky.distal.rotation.z,
          hand->pinky.distal.rotation.w,
          hand->pinky.metacarpal.rotation.x,
          hand->pinky.metacarpal.rotation.y,
          hand->pinky.metacarpal.rotation.z,
          hand->pinky.metacarpal.rotation.w,
          hand->pinky.proximal.rotation.x,
          hand->pinky.proximal.rotation.y,
          hand->pinky.proximal.rotation.z,
          hand->pinky.proximal.rotation.w,
          hand->pinky.intermediate.rotation.x,
          hand->pinky.intermediate.rotation.y,
          hand->pinky.intermediate.rotation.z,
          hand->pinky.intermediate.rotation.w,
	  hand->pinky.distal.next_joint.x,
          hand->pinky.distal.next_joint.y,
          hand->pinky.distal.next_joint.z,
          hand->pinky.metacarpal.next_joint.x,
          hand->pinky.metacarpal.next_joint.y,
          hand->pinky.metacarpal.next_joint.z,
          hand->pinky.proximal.next_joint.x,
          hand->pinky.proximal.next_joint.y,
          hand->pinky.proximal.next_joint.z,
          hand->pinky.intermediate.next_joint.x,
          hand->pinky.intermediate.next_joint.y,
          hand->pinky.intermediate.next_joint.z,
          hand->pinky.distal.prev_joint.x,
          hand->pinky.distal.prev_joint.y,
          hand->pinky.distal.prev_joint.z,
          hand->pinky.metacarpal.prev_joint.x,
          hand->pinky.metacarpal.prev_joint.y,
          hand->pinky.metacarpal.prev_joint.z,
          hand->pinky.proximal.prev_joint.x,
          hand->pinky.proximal.prev_joint.y,
          hand->pinky.proximal.prev_joint.z,
          hand->pinky.intermediate.prev_joint.x,
          hand->pinky.intermediate.prev_joint.y,
          hand->pinky.intermediate.prev_joint.z,
        };
        memcpy(data_send, data, sizeof(data));
        status = true;
      }
      char buffer_data[sizeof(data_send)];
      if(status==true)
      {
        printf("sending valid frame\n"); 
        memcpy(buffer_data, data_send, sizeof(data_send));
        n = send(newsockfd, buffer_data, sizeof(buffer_data), 0);
        if (n < 0) {
            perror("ERROR writing data to socket");
            exit(1);
        }
      }else{
        printf("invalid frame, not sending !\n");
      }
    }
    millisleep(50);
  } //ctrl-c to exit
  close(newsockfd);
  close(sockfd);
  return 0;
}
//End-of-Sample
