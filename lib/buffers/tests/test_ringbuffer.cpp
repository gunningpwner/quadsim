#include <gtest/gtest.h>
#include <vector>
#include "../include/ClaimCommitRingBuffer.h"

// Define a simple struct to test with
struct TestPacket {
    uint32_t id;
    float value;
    
    // overload == for easy GTest comparison
    bool operator==(const TestPacket& other) const {
        return id == other.id && value == other.value;
    }
};

// Test Fixture: automatically creates a buffer before every test
class RingBufferTest : public ::testing::Test {
protected:
    static constexpr size_t BUF_SIZE = 4;
    ClaimCommitRingBuffer<TestPacket, BUF_SIZE> buffer;
    CCRBNonCopyingConsumer<TestPacket, BUF_SIZE> consumer{buffer};
};

TEST_F(RingBufferTest, InitializationState) {
    // Consumer should start empty
    EXPECT_EQ(consumer.readNext(), nullptr);
    EXPECT_EQ(consumer.readLatest(), nullptr);
}

TEST_F(RingBufferTest, SingleClaimAndCommitLatest) {
    // 1. Claim a slot
    TestPacket* slot = buffer.claim();
    ASSERT_NE(slot, nullptr);
    
    // 2. Write data
    slot->id = 1;
    slot->value = 12.34f;
    
    // 3. Commit
    buffer.commit(slot);

    // 4. Verify Consumer sees it
    const TestPacket* read_ptr = consumer.readLatest();
    ASSERT_NE(read_ptr, nullptr);
    EXPECT_EQ(read_ptr->id, 1);
    EXPECT_EQ(read_ptr->value, 12.34f);
    
    // 5. Verify queue is now empty
    EXPECT_EQ(consumer.readLatest(), nullptr);
    
}
TEST_F(RingBufferTest, SingleClaimAndCommitNext) {
    // 1. Claim a slot
    TestPacket* slot = buffer.claim();
    ASSERT_NE(slot, nullptr);
    
    // 2. Write data
    slot->id = 1;
    slot->value = 12.34f;
    
    // 3. Commit
    buffer.commit(slot);

    // 4. Verify Consumer sees it
    const TestPacket* read_ptr = consumer.readNext();
    ASSERT_NE(read_ptr, nullptr);
    EXPECT_EQ(read_ptr->id, 1);
    EXPECT_EQ(read_ptr->value, 12.34f);
    
    // 5. Verify queue is now empty
    EXPECT_EQ(consumer.readNext(), nullptr);
    
}

TEST_F(RingBufferTest, BufferWrapAround) {
    // Fill the buffer (0, 1, 2, 3)
    for (uint32_t i = 0; i < BUF_SIZE; i++) {
        TestPacket* slot = buffer.claim();
        slot->id = i;
        buffer.commit(slot);
    }

    // Read all 4 items to empty the consumer
    for (uint32_t i = 0; i < BUF_SIZE; i++) {
        const TestPacket* p = consumer.readNext();
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->id, i);
    }
    
    // Now the head should be back at index 0 (wrapped).
    // Write one more packet (ID 100)
    TestPacket* slot = buffer.claim();
    slot->id = 100;
    buffer.commit(slot);
    
    // Read it back
    const TestPacket* p = consumer.readNext();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->id, 100);
}

TEST_F(RingBufferTest, OverwriteBehavior) {
    // Fill the buffer completely (IDs: 0, 1, 2, 3)
    for (uint32_t i = 0; i < BUF_SIZE; i++) {
        TestPacket* slot = buffer.claim();
        slot->id = i;
        buffer.commit(slot);
    }
    
    // Note: Consumer has NOT read anything yet.
    // The buffer is full. Now we force an overwrite (ID: 99).
    // This should push the "Tail" forward, deleting ID 0.
    TestPacket* slot = buffer.claim();
    slot->id = 99;
    buffer.commit(slot);

    // Now, let's see what the consumer reads.
    // It SHOULD skip ID 0 and start at ID 1.
    // Because your logic pushes the tail forward.
    
    // Depending on your "is_worried" logic, it might skip to the very end 
    // or just read from the new tail. 
    // Let's assume standard behavior: Read from new Tail (Index 1, ID 1)
    
    // Sync consumer (simulate the consumer realizing it was slow)
    // If your code handles this automatically inside readNext:
    const TestPacket* p = consumer.readNext();
    
    ASSERT_NE(p, nullptr);
    // We expect ID 0 to be gone. The next valid ID is 1.
    EXPECT_EQ(p->id, 1); 
    
    // Read the rest
    EXPECT_EQ(consumer.readNext()->id, 2);
    EXPECT_EQ(consumer.readNext()->id, 3);
    EXPECT_EQ(consumer.readNext()->id, 99); // The new one
    EXPECT_EQ(consumer.readNext(), nullptr); // Empty
}

TEST_F(RingBufferTest, ReadLatestLogic) {
    // Commit 3 packets: 10, 20, 30
    buffer.commit(buffer.claim()); // dummy commit to slot 0
    
    auto* s2 = buffer.claim(); s2->id = 20; buffer.commit(s2);
    auto* s3 = buffer.claim(); s3->id = 30; buffer.commit(s3);
    
    // readLatest() should skip everything and give us 30
    const TestPacket* latest = consumer.readLatest();
    ASSERT_NE(latest, nullptr);
    EXPECT_EQ(latest->id, 30);
    
    // Subsequent readNext should return null (caught up) or behave consistently
    // depending on implementation. Usually, readLatest moves the read pointer to head.
    EXPECT_EQ(consumer.readNext(), nullptr);
}

TEST_F(RingBufferTest, ResetLogic) {
    // Write data
    auto* s1 = buffer.claim(); s1->id = 55; buffer.commit(s1);
    
    // Consume it
    consumer.readNext();
    EXPECT_EQ(consumer.readNext(), nullptr);
    
    // Reset consumer pointer to tail
    consumer.reset();
    
    // Should be able to read it again (since it wasn't overwritten yet)
    const TestPacket* p = consumer.readNext();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->id, 55);
}

TEST_F(RingBufferTest, AvailableFunctionality) {
    // 1. Initial state
    EXPECT_EQ(consumer.available(), 0);

    // 2. Add items
    TestPacket* s1 = buffer.claim(); s1->id = 1; buffer.commit(s1);
    EXPECT_EQ(consumer.available(), 1);

    TestPacket* s2 = buffer.claim(); s2->id = 2; buffer.commit(s2);
    EXPECT_EQ(consumer.available(), 2);

    // 3. Read items
    consumer.readNext();
    EXPECT_EQ(consumer.available(), 1);

    consumer.readNext();
    EXPECT_EQ(consumer.available(), 0);
}

TEST_F(RingBufferTest, AvailableWithWrap) {
    // Fill buffer (size 4)
    for(uint32_t i=0; i<BUF_SIZE; i++) {
        auto* s = buffer.claim(); s->id = i; buffer.commit(s);
    }
    EXPECT_EQ(consumer.available(), 4);

    // Read 2
    consumer.readNext();
    consumer.readNext();
    EXPECT_EQ(consumer.available(), 2);

    // Add 1 (wrap)
    auto* s = buffer.claim(); s->id = 100; buffer.commit(s);
    // Now we have items at indices 2, 3, 0. (3 items)
    EXPECT_EQ(consumer.available(), 3);
    
    // Read all
    consumer.readNext(); // 2
    consumer.readNext(); // 3
    consumer.readNext(); // 0 (id 100)
    EXPECT_EQ(consumer.available(), 0);
}

TEST_F(RingBufferTest, ReadLatestWithWrap) {
    // Fill buffer
    for(uint32_t i=0; i<BUF_SIZE; i++) {
        auto* s = buffer.claim(); s->id = i; buffer.commit(s);
    }
    // Read latest (id 3)
    auto* p = consumer.readLatest();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->id, 3);

    // Add 1 (wrap)
    auto* s = buffer.claim(); s->id = 100; buffer.commit(s);
    
    // Read latest (id 100)
    p = consumer.readLatest();
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->id, 100);
}