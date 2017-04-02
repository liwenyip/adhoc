/*
 * File: Code for a new 'Dynamic Address Allocation' Agent Class for the ns
 *       network simulator
 * Author: Li-Wen Yip (LiWen.Yip@jcu.edu.au), September 2005
 *
 */


#include "daa.h"


/*
 * The following two static classes link the C++ classes with corresponding Tcl classes.
 */



int hdr_daa::offset_;
static class DaaHeaderClass : public PacketHeaderClass {
public:
	DaaHeaderClass() : PacketHeaderClass("PacketHeader/Daa", 
					      sizeof(hdr_daa)) {
		bind_offset(&hdr_daa::offset_);
	}
} class_daahdr;


static class DaaClass : public TclClass {
public:
	DaaClass() : TclClass("Agent/Daa") {}
	TclObject* create(int, const char*const*) {
		return (new DaaAgent());
	}
} class_daa;




/*
 * The constructor for the class 'DaaAgent'.
 * It binds the variables which have to be accessed both in Tcl and C++.
 */
DaaAgent::DaaAgent() : Agent(PT_DAA)
{
  bind("packetSize_", &size_);
}

/*
 * The function 'command()' is called when a Tcl command for the class 'DaaAgent' is executed.
 */
int DaaAgent::command(int argc, const char*const* argv)
{
    if (argc == 2) {
        if (strcmp(argv[1], "init") == 0) {
            // Run the initialisation procedure
			state_ = INIT;
			my_addr[0] = 0;
			my_addr[1] = 0;
			areq_retry_ = 0;
			arep_counter_ = 0;
			nrep_counter_ = 0;
			best_offer_ = NULL;
            init();
            return(TCL_OK);
        }
  }
  // If the command hasn't been processed by DaaAgent()::command,
  // call the command() function for the base class
  return (Agent::command(argc, argv));
}


void DaaAgent::recv(Packet* pkt, Handler*)
{
	// Access the IP header for the received packet:
	hdr_ip* hdrip = hdr_ip::access(pkt);
	// Access the DAA header for the received packet:
	hdr_daa* hdr = hdr_daa::access(pkt);
	
	// Check the packet type and pass it to the appropriate function.
	if 	    (hdr->type == AREQ)		recv_areq(pkt);
	else if (hdr->type == AREP)		recv_arep(pkt);
	else if (hdr->type == NREP)		recv_nrep(pkt);
	else if (hdr->type == AACK)		recv_aack(pkt);
//	else if (hdr->type == PACK)		recv_pack(pkt);
//	else if (hdr->type == ASRCH)	recv_asrch(pkt);

    // Discard the packet once it's been processed
    Packet::free(pkt);
}

//////////////////////////////////////////////////////////////////////////////
// CONTROLLER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

//
// Controls the initialisation procedure
//
void DaaAgent::init()
{
	// If we aren't in the INIT state, then bugger off
	if (state_ != INIT) return;
	
    // Check if we have received any replies.
    // If we have, then accept the best offer and enter the IDLE state.
    if (arep_counter_ > 0)
    {
        // Create a reply to the best offer and send it
        Packet* reply = create_reply(AACK, best_offer_);
        send(reply, 0);
        
        // Take posession of the offered addresses
        hdr_daa* hdr = hdr_daa::access(best_offer_);
        my_addr[0] = hdr->alloc_addr[0];
        my_addr[1] = hdr->alloc_addr[1];
        
        // Dispose of the packet
        Packet::free(best_offer_);
        
        // Enter the idle state
        state_ = IDLE;
        return;
    }
	
	// We haven't received any replies, so check how many times we have broadcast AREQ.
	// If it is less than the limit, (re)broadcast the AREP message, and schedule a retry.
    else if (areq_retry_ <= AREQ_LIMIT_)
    {
	    // Send an AREQ broadcast packet, and include a sequence number
        Packet* request = create_broadcast(AREQ, areq_retry_++);
	    send(request, 0);
	    
	    // Schedule a timeout
	    // ### TO DO ###
	    
	    return;
    }
    
    // We have exceeded the retry limit, so assume we are not in range of an existing network
    // and take posession of the entire address space.
    else
    {
        // Take posession of the entire address space
        my_addr[0] = 1;
        my_addr[1] = 254;
        
        // Enter the idle state
        state_ = IDLE;
        return;
    }


}

//
// Controls the allocation procedure
//
void DaaAgent::alloc()
{
	// If we aren't in the ALLOC state, bugger off.
	if (state_ != ALLOC) return;
	
	// If we are still in the ALLOC state then we haven't had a reply.
	// If we haven't reached the retry limit, (re)transmit an AREP message.
	else if (arep_retry_ <= AREP_LIMIT_)
	{
		// Send an AREP reply to the AREQ source packet
        Packet* reply = create_reply(AREP, areq_src_);
        send(reply, 0);
        
        // Increase the retry counter
        arep_retry_++;
        
        // Schedule a timeout
		// ## TO DO ##
	}
	
	// If we have reached the retry limit, give up and go back to the IDLE state.
	else
	{
		Packet::free(areq_src_);
		state_ = IDLE;
	}
		
}

//////////////////////////////////////////////////////////////////////////////
// PACKET RECEIVING FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

void DaaAgent::recv_areq(Packet* pkt)
{
	// If we are not in the IDLE state, discard the packet
	if (state_ != IDLE) 
	{
		Packet::free(pkt);
		return;
	}

	// Check if we have any available addresses
	else if (my_addr[1] > my_addr[0])
	{
		// We have available addresses: allocate the upper half.
        alloc_addr[1] = my_addr[1];
        alloc_addr[0] = (my_addr[0] + my_addr[1]) / 2;
        
        // Start the allocation procedure
        arep_retry_ = 0;		// Reset the retry counter
        areq_src_ = pkt;		// Save the AREQ packet
		alloc();
		return;
	}

	// We don't have available addresses, if this is only the first request then
	// discard the packet.
	else if (hdr_daa::access(pkt)->seq == 0)
	{
		Packet::free(pkt);
	}

	// If this is not the first request, then send a negative reply and go into 
	// the proxy state.
	else
	{
		
		// Send the NREP
		Packet* reply = create_reply(NREP, pkt);
		send(reply, 0);
		
		// Enter the proxy state
		Packet::free(pkt);

		return;
	}
}

// Process an AREP packet
void DaaAgent::recv_arep(Packet* pkt)
{
    // If we are not in initialisation mode, discard the packet.
    if (state_ != INIT)
    {
    	Packet::free(pkt);
    }

	// If the uid doesn't match, discard the packet.
	if (hdr_daa::access(pkt)->uid != uid)
	{
		Packet:free(pkt);
	}

    // If it is better than our previous best offer, discard the
    // previous best offer and save the new one
    else
    {
        arep_counter_++;
        hdr_daa* hdr = hdr_daa::access(pkt);
        int new_offer_size = hdr->alloc_addr[1] - hdr->alloc_addr[0];
        if (new_offer_size > best_offer_size_ || best_offer_ == NULL)
        {
        	Packet::free(best_offer_);
        	best_offer_ = pkt;
        	best_offer_size_ = new_offer_size;
        }
        else
        {
        	Packet::free(pkt);
        }
        return;
    }
    
}

// Process an NREP packet
void DaaAgent::recv_nrep(Packet* pkt)
{
    // Discard if we are not in initialisation mode.
    if (state_ != INIT)
    {
    	Packet::free(pkt);
    }
    else
    {
    	nrep_counter_++;
    	// save the address so we can track which addresses are in use.
    	// ## TO DO ##
    	Packet::free(pkt);
    }
}

// Process an AACK packet
void DaaAgent::recv_aack(Packet* pkt)
{
    // Discard if we are not in allocation mode.
    if (state_ != ALLLOC)
    {
    	Packet::free(pkt);
    }
    else
    {
    	// Check the UID on the packet:
    	hdr_daa* hdr = hdr_daa::access(pkt);
    	if (hdr->uid == 
    }
}




//////////////////////////////////////////////////////////////////////////////
// PACKET FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

//
// Creates a new broadcast with the specified type and sequence number.
//
Packet* DaaAgent::create_broadcast(int type, int seq)
{
	// Create a new packet
	Packet* pkt = allocpkt();

	// Populate the header with type, sequence number, and UID
	hdr_daa* hdr = hdr_daa::access(pkt);
	hdr->type = type;
	hdr->seq = seq;
	hdr->uid = uid_;

  	// Set the destination address to broadcast in the IP header
  	hdr_ip* iphdr = hdr_ip::access(pkt);
  	iphdr->daddr() = IP_BROADCAST;
	iphdr->dport() = iphdr->sport();
	
	// Give it back
	return pkt;
}

//
// Creates a reply to the specified packet of the specified type
//
Packet* DaaAgent:: create_reply(int type, Packet* src)
{
	// Create a new packet
	Packet* pkt = allocpkt();

	// Copy the UID from the old packet into the new packet, and set the type
	hdr_daa* dest_hdr = hdr_daa::access(pkt);
	hdr_daa* src_hdr = hdr_daa::access(src);
	dest_hdr->uid = src_hdr->uid;
	dest_hdr->type = type;

  	// Copy saddr from the old packet into daddr of the new packet
  	hdr_ip* dest_iphdr = hdr_ip::access(pkt);
  	hdr_ip* src_iphdr = hdr_ip::access(src);
  	dest_iphdr->daddr() = src_iphdr->saddr();
	dest_iphdr->dport() = src_iphdr->sport();
	
	// Give it back
	return pkt;
}
