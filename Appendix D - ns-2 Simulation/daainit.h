/*
    Initialisation Controller for Dynamic Address Allocation
    Li-Wen Yip <liwen.yip@jcu.edu.au>, 20 September 2005
*/
 
#ifndef ns_daa_init_h
#define ns_daa_init_h

#include "agent.h"
#include "tclcl.h"
#include "packet.h"
#include "address.h"
#include "ip.h" 

#include "daa.h"

class DaaInit {

    // Parent
    DaaAgent* parent;
        
    // AREQ retry counters  
    int areq_counter_ =;            // AREQ retry counter
    static const int AREQ_LIMIT_;   // AREQ retry limit
    static const int AREQ_TIMEOUT_; // AREQ retry timeout;

    // AREP / NREP received counters
    int arep_counter_;              // AREP retry/received counter
    int nrep_counter_;              // NREP received counter
    Packet* best_offer_;            // Best offer

    // Default constructor
    DaaInit(DaaAgent* /* parent */);
    
    // Run the routine
    void run();
    
    // Callbacks
    void recv(Packet*);
    
};

#endif