// SPDX-License-Identifier: MIT
#include "core.h"
#include "shape.h"
#include "test_macros.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include <stdlib.h>
#include <string.h>

static int SubsetTest( void )
{
	b3MeshData* source = b3CreateGridMesh( 14, 12, 0.2f, 3, true );
	ENSURE( source != NULL );
	uint8_t* backup = malloc( source->byteCount );
	memcpy( backup, source, source->byteCount );
	int invalid[] = { -1, source->triangleCount }, duplicate[] = { 0, 0 };
	ENSURE( b3CreateMeshSubset( NULL, duplicate, 1 ) == NULL );
	ENSURE( b3CreateMeshSubset( source, NULL, 1 ) == NULL );
	ENSURE( b3CreateMeshSubset( source, duplicate, 0 ) == NULL );
	ENSURE( b3CreateMeshSubset( source, duplicate, 2 ) == NULL );
	ENSURE( b3CreateMeshSubset( source, invalid, 1 ) == NULL );
	ENSURE( b3CreateMeshSubset( source, invalid + 1, 1 ) == NULL );
	b3MeshData* pieces[16]; int pieceCount = 0, total = 0, flat = 0;
	for ( int start = 0; start < source->triangleCount; start += 37 )
	{
		int selection[37], count = b3MinInt( 37, source->triangleCount - start );
		for ( int i = 0; i < count; ++i ) selection[i] = start + count - 1 - i;
		b3MeshData* part = b3CreateMeshSubset( source, selection, count );
		ENSURE( part != NULL && part->triangleCount == count );
		ENSURE( part->bounds.upperBound.x - part->bounds.lowerBound.x <=
			source->bounds.upperBound.x - source->bounds.lowerBound.x );
		uint64_t hash = part->hash;
		part->hash = 0;
		ENSURE( b3Hash64NonZero( (uint8_t*)part, part->byteCount ) == hash );
		part->hash = hash;
		const b3MeshTriangle* a = b3GetMeshTriangles( part ), *b = b3GetMeshTriangles( source );
		bool seen[37] = { 0 };
		for ( int t = 0; t < count; ++t )
		{
			int found = -1;
			for ( int j = 0; j < count; ++j )
				if ( !seen[j] && !memcmp( &a[t], &b[selection[j]], sizeof( b3MeshTriangle ) ) ) { found = j; break; }
			ENSURE( found >= 0 ); seen[found] = true;
			ENSURE( b3GetMeshFlags( part )[t] == b3GetMeshFlags( source )[selection[found]] );
			ENSURE( b3GetMeshMaterialIndices( part )[t] == b3GetMeshMaterialIndices( source )[selection[found]] );
			flat += b3GetMeshFlags( part )[t] != 0;
		}
		pieces[pieceCount++] = part; total += count;
	}
	ENSURE( flat > 0 && total == source->triangleCount );
	b3Mesh original = { source, { 1, 1, 1 } };
	for ( int i = 0; i < 30; ++i ) for ( int j = 0; j < 30; ++j )
	{
		b3RayCastInput ray = { .origin = { (i + .37f) * 2.8f / 30, 1, (j + .61f) * 2.4f / 30 },
			.translation = { 0, -2, 0 }, .maxFraction = 1 };
		b3CastOutput full = b3RayCastMesh( &original, &ray ), selected = { .fraction = 1 };
		for ( int p = 0; p < pieceCount; ++p )
		{
			b3Mesh mesh = { pieces[p], { 1, 1, 1 } }; b3CastOutput hit = b3RayCastMesh( &mesh, &ray );
			if ( hit.hit && ( !selected.hit || hit.fraction < selected.fraction ) ) selected = hit;
		}
		ENSURE( full.hit == selected.hit );
		if ( full.hit ) ENSURE_SMALL( full.fraction - selected.fraction, 1e-6f );
	}
	ENSURE( !memcmp( backup, source, source->byteCount ) );
	free( backup ); b3DestroyMesh( source );
	// Subsets own their vertices and flags, independently of their source lifetime.
	for ( int p = 0; p < pieceCount; ++p )
	{
		ENSURE( b3IsValidMesh( pieces[p] ) ); b3DestroyMesh( pieces[p] );
	}
	return 0;
}

static int CurvedSubsetTest( void )
{
	b3MeshData* source = b3CreateTorusMesh( 24, 12, 1, .2f );
	ENSURE( source && source->triangleCount > 256 );
	b3MeshData* pieces[32]; int pieceCount = 0, edges = 0;
	const b3MeshTriangle* original = b3GetMeshTriangles( source );
	for ( int start = 0; start < source->triangleCount; start += 48 )
	{
		int selection[48], count = b3MinInt( 48, source->triangleCount - start );
		for ( int i = 0; i < count; ++i ) selection[i] = start + i;
		b3MeshData* part = b3CreateMeshSubset( source, selection, count );
		ENSURE( part && part->triangleCount == count );
		const b3MeshTriangle* triangles = b3GetMeshTriangles( part );
		for ( int t = 0; t < count; ++t )
		{
			int found = -1;
			for ( int s = start; s < start + count; ++s )
				if ( !memcmp( &triangles[t], &original[s], sizeof( b3MeshTriangle ) ) ) { found = s; break; }
			ENSURE( found >= 0 );
			ENSURE( b3GetMeshFlags( part )[t] == b3GetMeshFlags( source )[found] );
			edges += b3GetMeshFlags( part )[t] != 0;
		}
		pieces[pieceCount++] = part;
	}
	ENSURE( edges > 0 );
	b3Mesh full = { source, { 1, 1, 1 } };
	for ( int axis = 0; axis < 3; ++axis ) for ( int i = 0; i < 20; ++i ) for ( int j = 0; j < 20; ++j )
	{
		float v[3] = { 0 }, delta[3] = { 0 };
		v[axis] = -2; delta[axis] = 4;
		v[(axis + 1) % 3] = -1.25f + (i + .37f) * 2.5f / 20;
		v[(axis + 2) % 3] = -1.25f + (j + .61f) * 2.5f / 20;
		b3RayCastInput ray = { .origin = { v[0], v[1], v[2] }, .translation = { delta[0], delta[1], delta[2] }, .maxFraction = 1 };
		b3CastOutput a = b3RayCastMesh( &full, &ray ), b = { .fraction = 1 };
		for ( int p = 0; p < pieceCount; ++p )
		{
			b3Mesh mesh = { pieces[p], { 1, 1, 1 } }; b3CastOutput hit = b3RayCastMesh( &mesh, &ray );
			if ( hit.hit && ( !b.hit || hit.fraction < b.fraction ) ) b = hit;
		}
		ENSURE( a.hit == b.hit ); if ( a.hit ) ENSURE_SMALL( a.fraction - b.fraction, 1e-6f );
	}
	for ( int p = 0; p < pieceCount; ++p ) b3DestroyMesh( pieces[p] );
	b3DestroyMesh( source );
	return 0;
}

int MeshSubsetTest( void )
{
	RUN_SUBTEST( SubsetTest );
	RUN_SUBTEST( CurvedSubsetTest );
	return 0;
}
