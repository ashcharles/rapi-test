#pragma once

#include <map>
#include "tinyjson.hpp"
#include "network_server_udp.h"
#include "network_client_udp.h"


class JsonRpcServer
{
	public:
		typedef json::grammar<char>::variant variant;
		typedef json::grammar<char>::object object;
		typedef json::grammar<char>::array array;

		///helper function to wrap anything into a tinyjson variant type
		template< typename ValueType > static variant toVariant( const ValueType& value ) { return variant( new boost::any( value ) ); }


	private:
		class RPCMethodAbstractBase {
			public:
				virtual void call( variant args, object& responce ) = 0;
		};

		std::map< std::string, RPCMethodAbstractBase* > _map;

		//function taken from jsoncpp (public domain license)
		static std::string valueToQuotedString( const char *value )
		{
			// Not sure how to handle unicode...
			if (strpbrk(value, "\"\\\b\f\n\r\t") == NULL)
				return std::string("\"") + value + "\"";
			// We have to walk value and escape any special characters.
			// Appending to std::string is not efficient, but this should be rare.
			// (Note: forward slashes are *not* rare, but I am not escaping them.)
			unsigned maxsize = strlen(value)*2 + 3; // allescaped+quotes+NULL
			std::string result;
			result.reserve(maxsize); // to avoid lots of mallocs
			result += "\"";
			for (const char* c=value; *c != 0; ++c){
				switch(*c){
					case '\"':
						result += "\\\"";
					break;
					case '\\':
					result += "\\\\";
					break;
					case '\b':
					result += "\\b";
					break;
					case '\f':
					result += "\\f";
					break;
					case '\n':
					result += "\\n";
					break;
					case '\r':
					result += "\\r";
					break;
					case '\t':
					result += "\\t";
					break;
					case '/':
					// Even though \/ is considered a legal escape in JSON, a bare
					// slash is also legal, so I see no reason to escape it.
					// (I hope I am not misunderstanding something.)
					default:
					result += *c;
				}
			}
			result += "\"";
			return result;
		}


	public:
		static std::string toString( const variant& v )
		{
			if( v->empty() == true ) {
				return "null";
			}
			else if( v->type() == typeid( bool ) )
			{
				// variant is of type "bool"...
				bool b = boost::any_cast< bool >(*v);
				return ( b ? "true" : "false" );
			}
			else if( v->type() == typeid( int ) )
			{
				// variant is of type "int"...
				int i = boost::any_cast< int >(*v);
				std::ostringstream oss;
				oss << i;
				return oss.str();
			}
			else if( v->type() == typeid(double))
			{
				// variant is of type "double"...
				double d = boost::any_cast< double >(*v);
				std::ostringstream oss;
				oss << d;
				return oss.str();
			}
			else if( v->type() == typeid(std::string))
			{
				// variant is a string...
				std::string s = boost::any_cast< std::string >(*v);
				return valueToQuotedString( s.c_str() );
			}
			else if( v->type() == typeid(json::grammar<char>::array))
			{
				// variant is an array => use recursion
				json::grammar<char>::array const & a = boost::any_cast< json::grammar<char>::array >(*v);

				std::ostringstream oss;
				oss << "[";
				for(json::grammar<char>::array::const_iterator it = a.begin(); it != a.end(); ++it)
				{
					if( it != a.begin() )
						oss << ",";
					oss << toString( *it );
				}
				oss << "]";
				return oss.str();
			}
			else if( v->type() == typeid(json::grammar<char>::object))
			{
				// variant is an object => use recursion
				json::grammar<char>::object const & o = boost::any_cast< json::grammar<char>::object >(*v);

				std::ostringstream oss;
				oss << "{";
				for(json::grammar<char>::object::const_iterator it = o.begin(); it != o.end(); ++it)
				{
					if( it != o.begin() )
						oss << ",";
					std::string key = (*it).first;
					oss << valueToQuotedString( key.c_str() ) << ":" << toString( it->second );
				}
				oss << "}";
				return oss.str();
			}
			else
			{
				assert( 0 );
				// ERROR: unknown type...
			}
		}

	public:


		template< class T > 
		class RPCMethod : public RPCMethodAbstractBase {
			public:
				typedef void (T::*method)(JsonRpcServer::variant, JsonRpcServer::object& );
				RPCMethod( T *inst, method m ) : _inst( inst ), _meth( m ) {}
				virtual void call( variant args, object& responce ) { (_inst->*_meth)( args, responce ); }

			private:
				T *_inst;
				method _meth;
		};

		void addMethodHandler( RPCMethodAbstractBase* m, const std::string& method_name ) {
			if( _map[ method_name ] != NULL )
				delete _map[ method_name ];
			_map[ method_name ] = m;
		}

	protected:
		std::string call( const std::string& json_str )
		{
			variant v = json::parse( json_str.begin(), json_str.end() );
			assert( v->empty() == false );
			//TODO replace with JSONRPC error return str
			assert( v->type() == typeid( object) );
			object o = boost::any_cast< object >( *v );

			variant method_v = o[ "method" ];
			assert( method_v->type() == typeid( std::string ) );
			std::string method = boost::any_cast< std::string >( *method_v );

			RPCMethodAbstractBase *m = _map[ method ];
			assert( m != NULL );
			object responce;
			m->call( o[ "params" ], responce );

			return toString( variant(new boost::any( responce ) ) );

		}
};

class JsonRpcUDPServer : public JsonRpcServer
{
	private:
		NetworkServerUDP _server;
		NetworkServerUDP::ReceivedPacket _received_packet;
	public:
		JsonRpcUDPServer( int port ) : _server( port ) { }
		bool recv( void )
		{
				bool received = _server.recv( _received_packet );
				if( received == true ) {
					std::string responce = call( _received_packet.s );
					_server.send( responce.c_str(), &(_received_packet.addr) );
				}
				return received;
		}
};

class JsonRpcUDPClient 
{
	private:
		NetworkClientUDP _client;
		NetworkServerUDP::ReceivedPacket _received_packet;
	public:
		JsonRpcUDPClient( std::string server_ip, int port ) : _client( server_ip, port ) { }
		void call( JsonRpcServer::variant v )
		{
			_client.send( JsonRpcServer::toString( v ) );
		}

		bool recv( JsonRpcServer::variant& var )
		{
			bool received = _client.recv( _received_packet );
			if( received == false )
				return false;

			var = json::parse( _received_packet.s.begin(), _received_packet.s.end() );
			return true;
		}
};

