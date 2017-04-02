/*
 * File: Code for a new 'Dynamic Address Allocation' Agent Class for the ns
 *       network simulator
 * Author: Li-Wen Yip (LiWen.Yip@jcu.edu.au), September 2005
 *
 */


#ifndef ns_daa_h
#define ns_daa_h

#include "agent.h"
#include "tclcl.h"
#include "packet.h"
#include "address.h"
#include "ip.h"


/*
 * Packet Types:
 * AREQ = Address Request
 * AREP = Address Reply (Offer)
 * NREP = Negative Reply
 * AACK = Address Accept
 * PACK = Proxy Address Accept
 * ASRCH = Initiate Address Search
 */

enum {AREQ, AREP, NREP, AACK, PACK, ASRCH};


/*
 * Agent States:
 * UNINIT = Uninitialised
 * INIT = Initialising
 * IDLE = Idle
 * ALLOC = Allocating addresses
 * PROXY = Acting as an allocation proxy
 */

enum {UNINIT, INIT, IDLE, ALLOC, PROXY};


/*
 * The data structure for the Dynamic Address Allocation packet header
 */
struct hdr_daa {
    short type;         // The message type
    int seq;            // The sequence number
    int uid;            // The unique ID of the requesting node
    int alloc_addr[2];  // The address range being allocated

    // Header access methods
    static int offset_; // required by PacketHeaderManager
    inline static int& offset() { return offset_; }
    inline static hdr_daa* access(const Packet* p) {
        return (hdr_daa*) p->access(offset_);
    }
};

/*
 * Define the Dynamic Address Allocation agent as a subclass of "Agent"
 */
class DaaAgent : public Agent {
 public:
    // Default Constructor
    DaaAgent();
    // Execute a command
    int command(int argc, const char*const* argv);
    // Process a packet
    void recv(Packet*, Handler*);
    

    // Agent Variables
    int state_;						// Agent State
    int uid_;                       // Unique identifier;
    int my_addr[2];                 // My Address Space
 
    // Variables for Initialisation
    int areq_retry_ =;            	// AREQ retry counter
    static const int AREQ_LIMIT_;   // AREQ retry limit
    static const int AREQ_TIMEOUT_; // AREQ retry timeout
    int arep_counter_;              // AREP retry/received counter
    int nrep_counter_;              // NREP received counter
    Packet* best_offer_;            // Best offer
    int best_offer_size_;			// Size of the best offer
    
	// Variables for Allocation
	int arep_retry_					// AREP retry counter
	static const int AREP_LIMIT_;	// AREP retry limit
	static const int AREQ_TIMEOUT_;	// AREP retry timeout;
    int alloc_addr[2];              // Allocated Address Space
    Packet* areq_src_;				// the AREQ we are replying to
    
    // Controller functions
    void init();
    void alloc();
    void recv_areq(Packet*);
    void recv_arep(Packet*);
    void recv_nrep(Packet*);
    void recv_aack(Packet*);

    // Packet creation functions
    static Packet* create_broadcast(int /*type*/, int /*seq*/);
    static Packet* create_reply(int /*type*/, Packet* /*src*/);
    
};


#endif
